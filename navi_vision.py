"""NAVI vision: webcam capture, red / green detection and the UI overlay.

The vision thread detects; the UI thread draws, so a slow redraw never stalls detection.

    python3 navi_vision.py                  webcam and window
    python3 navi_vision.py --headless       no window, saves frames
    python3 navi_vision.py --source images  cycle test_images/
"""

import argparse
import datetime
import glob
import os
import threading
import time

# Draw on the Pi's own screen even when started over SSH. X11 owns the
# display, so point at it unless the caller already did.
os.environ.setdefault("DISPLAY", ":0")

import cv2
import numpy as np

import navi_config as cfg


# Direct framebuffer output for the ILI9486 (--lcd), for running without a desktop.

class LCDWriter:
    """Write OpenCV frames to the SPI LCD framebuffer."""

    def __init__(self, fb="/dev/fb0"):
        self.fb = fb
        self.w, self.h, self.bpp = self._geometry()
        # Unbind the text console so it stops drawing over fb0.
        for vtcon in sorted(os.listdir("/sys/class/vtconsole/")):
            bind_path = "/sys/class/vtconsole/%s/bind" % vtcon
            try:
                with open(bind_path, "w") as fh:
                    fh.write("0")
            except PermissionError:
                os.system("sudo sh -c 'echo 0 > %s'" % bind_path)
            except Exception:
                pass
        self._fh = None

    def _geometry(self):
        base = "/sys/class/graphics/%s" % os.path.basename(self.fb)
        try:
            with open(base + "/virtual_size") as fh:
                w, h = (int(v) for v in fh.read().strip().split(","))
            with open(base + "/bits_per_pixel") as fh:
                bpp = int(fh.read().strip())
            return w, h, bpp
        except Exception:
            return 320, 480, 16

    def write(self, frame):
        """Write an OpenCV BGR frame to the LCD, resized to fit."""
        if self._fh is None:
            try:
                self._fh = open(self.fb, "wb")
            except PermissionError:
                return
        resized = cv2.resize(frame, (self.w, self.h))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        r = (rgb[:, :, 0].astype(np.uint16) >> 3) << 11
        g = (rgb[:, :, 1].astype(np.uint16) >> 2) << 5
        b = rgb[:, :, 2].astype(np.uint16) >> 3
        pixels = (r | g | b).astype("<u2")
        try:
            self._fh.seek(0)
            self._fh.write(pixels.tobytes())
            self._fh.flush()
        except Exception:
            pass

    def close(self):
        if self._fh:
            try:
                self._fh.close()
            except Exception:
                pass
            self._fh = None


# Detection core

def normalize(frame):
    """Shrink big frames to a fixed width so area thresholds mean the same for every source."""
    h, w = frame.shape[:2]
    if w > cfg.PROC_WIDTH:
        scale = cfg.PROC_WIDTH / float(w)
        frame = cv2.resize(frame, (cfg.PROC_WIDTH, int(round(h * scale))),
                           interpolation=cv2.INTER_AREA)
    return frame


def clean_mask(mask):
    """Open (remove speckle) then close (fill holes) the mask."""
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (cfg.MORPH_KERNEL, cfg.MORPH_KERNEL))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def largest_object(mask, min_area):
    """Biggest blob at or above min_area as (contour, area), or (None, 0)."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0
    for c in contours:
        a = cv2.contourArea(c)
        if a >= min_area and a > best_area:
            best, best_area = c, a
    return best, best_area


def steering_packet(green_contour, frame_w):
    """Turn the green target's horizontal offset into left and right PWM (tank mixing)."""
    M = cv2.moments(green_contour)
    if M["m00"] == 0:
        return 0, 0, (0, 0)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    # error from minus 1 to plus 1, negative means target is left of center
    error = (cx - frame_w / 2.0) / (frame_w / 2.0)
    turn = int(cfg.TURN_GAIN * error * cfg.PWM_MAX)

    left = int(np.clip(cfg.BASE_SPEED + turn, -cfg.PWM_MAX, cfg.PWM_MAX))
    right = int(np.clip(cfg.BASE_SPEED - turn, -cfg.PWM_MAX, cfg.PWM_MAX))
    return left, right, (cx, cy)


def detect(frame, depth=None):
    """Turn one normalized frame into a drive decision.

        Red is checked before green. Nothing found means STOP. depth (from
        navi_depth.zone_map) can only add a stop.
    """
    h, w = frame.shape[:2]
    min_area = cfg.MIN_AREA_FRAC * (w * h)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    green_mask = clean_mask(cv2.inRange(hsv, cfg.LOWER_GREEN, cfg.UPPER_GREEN))
    red_mask = clean_mask(cv2.bitwise_or(
        cv2.inRange(hsv, cfg.LOWER_RED_1, cfg.UPPER_RED_1),
        cv2.inRange(hsv, cfg.LOWER_RED_2, cfg.UPPER_RED_2)))

    green_c, green_area = largest_object(green_mask, min_area)
    red_c, red_area = largest_object(
        red_mask, getattr(cfg, "RED_MIN_AREA_FRAC", cfg.MIN_AREA_FRAC) * (w * h))

    depth_blocked = depth is not None and depth.get("path_blocked", False)

    # Red beats green, safety first. Flag is 1 on a hazard.
    if red_c is not None:
        command, status, color = "STOP", "RED HAZARD DETECTED", (0, 0, 255)
        left, right, flag, centroid = 0, 0, 1, None
    elif depth_blocked:
        command = "STOP"
        status = "DEPTH OBSTACLE " + depth["reason"]
        color = (0, 0, 255)
        left, right, flag, centroid = 0, 0, 1, None
    elif green_c is not None:
        left, right, centroid = steering_packet(green_c, w)
        command = "MOVE FORWARD"
        # name the turn so the printed table is readable
        if left > right + 10:
            status = "GREEN TARGET STEER RIGHT"
        elif right > left + 10:
            status = "GREEN TARGET STEER LEFT"
        else:
            status = "GREEN TARGET STRAIGHT"
        color, flag = (0, 255, 0), 0
    else:
        command, status, color = "STOP", "NO TARGET DETECTED", (0, 165, 255)
        left, right, flag, centroid = 0, 0, 0, None

    packet = "<%d,%d,%d>" % (left, right, flag)
    return {
        "command": command, "status": status, "color": color,
        "packet": packet, "flag": flag,
        "left": left, "right": right,
        "green_mask": green_mask, "red_mask": red_mask,
        "green_c": green_c, "red_c": red_c,
        "green_area": int(green_area), "red_area": int(red_area),
        "centroid": centroid,
        "depth": depth,
    }


def annotate(frame, d, hud=None):
    """Draw the overlay on a copy of the frame."""
    frame = frame.copy()
    h, w = frame.shape[:2]

    # center line, the steering reference
    cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)

    # bounding boxes on detected objects
    if d["red_c"] is not None:
        x, y, bw, bh = cv2.boundingRect(d["red_c"])
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
        cv2.putText(frame, "RED", (x, max(y - 8, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if d["green_c"] is not None:
        x, y, bw, bh = cv2.boundingRect(d["green_c"])
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.putText(frame, "GREEN", (x, max(y - 8, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if d["centroid"] is not None:
        cv2.circle(frame, d["centroid"], 6, (0, 255, 0), -1)

    # heads up display text
    cv2.putText(frame, "COMMAND: " + d["command"], (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, d["color"], 2)
    cv2.putText(frame, d["status"], (15, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, d["color"], 2)
    cv2.putText(frame, "UART: " + d["packet"], (15, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    if hud:
        cv2.putText(frame, hud, (15, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    # depth zone strip along the bottom when depth is on
    if d.get("depth"):
        _draw_depth_strip(frame, d["depth"])
    return frame


def _draw_depth_strip(frame, depth):
    """Three coloured blocks showing the left, center and right ToF verdicts."""
    h, w = frame.shape[:2]
    zones = depth.get("zones", {})
    names = ("left", "center", "right")
    colors = {"CLEAR": (0, 160, 0), "CAUTION": (0, 165, 255),
              "BLOCKED": (0, 0, 255), "UNKNOWN": (80, 80, 80)}
    band_h, y0 = 26, h - 60
    for i, name in enumerate(names):
        z = zones.get(name)
        if z is None:
            continue
        x0 = int(i * w / 3.0)
        x1 = int((i + 1) * w / 3.0)
        cv2.rectangle(frame, (x0, y0), (x1, y0 + band_h),
                      colors.get(z["verdict"], (80, 80, 80)), -1)
        mm = z["distance_mm"]
        label = "%s %s" % (name[0].upper(), "n/a" if mm is None else "%dmm" % mm)
        cv2.putText(frame, label, (x0 + 6, y0 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)


# Frame sources

class CameraSource:
    """The USB webcam through V4L2."""

    def __init__(self, index=None, width=None, height=None, fps=None):
        index = cfg.CAMERA_INDEX if index is None else index
        backend = getattr(cv2, "CAP_V4L2", 0) if os.name != "nt" else 0
        if backend:
            self.cap = cv2.VideoCapture(index, backend)
        else:
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(
                "Could not open camera index %s. Check ls /dev/video* and that "
                "no other process is holding the webcam." % index)
        # MJPG matters. At 720p30 raw YUYV fills the USB bandwidth and the
        # driver quietly drops to a much lower frame rate.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width or cfg.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height or cfg.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, fps or cfg.CAMERA_FPS)
        # Two buffers: one gives 15 fps on the Pi 5, two gives 30 fps.
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        except Exception:
            pass
        # Keep 30 fps in dim light: fixed frame rate, gain instead of longer exposure. Needs v4l2-ctl.
        if os.name != "nt":
            try:
                import subprocess
                subprocess.run(["v4l2-ctl", "-d", "/dev/video%d" % index,
                                "-c", "exposure_dynamic_framerate=0"],
                               capture_output=True, timeout=3)
            except Exception:
                pass

    def read(self):
        ok, frame = self.cap.read()
        return (frame, "camera") if ok else (None, None)

    def describe(self):
        fps = int(self.cap.get(cv2.CAP_PROP_FPS)) or cfg.CAMERA_FPS
        return "EMEET webcam %dx%d at %d fps" % (
            int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), fps)

    def release(self):
        self.cap.release()


class ImageSource:
    """Loop test_images/ as a fake camera."""

    def __init__(self, directory=None, loop=True):
        directory = directory or cfg.INPUT_DIR
        patterns = ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG", "*.PNG")
        paths = []
        for p in patterns:
            paths.extend(glob.glob(os.path.join(directory, p)))
        paths = sorted(set(paths))
        if not paths:
            raise RuntimeError("No images found in %s" % directory)
        self.frames = []
        for path in paths:
            raw = cv2.imread(path)
            if raw is not None:
                self.frames.append((os.path.basename(path), raw))
        if not self.frames:
            raise RuntimeError("No readable images in %s" % directory)
        self.i = 0
        self.loop = loop

    def read(self):
        if self.i >= len(self.frames) and not self.loop:
            return None, None
        name, frame = self.frames[self.i % len(self.frames)]
        self.i += 1
        return frame.copy(), name

    def describe(self):
        return "simulated stream, %d image(s)" % len(self.frames)

    def release(self):
        pass


# Single slot buffer

class LatestOnly:
    """One-slot mailbox: a new item replaces the pending one."""

    def __init__(self):
        self._item = None
        self._seq = 0                       # bumps on every put
        self._lock = threading.Lock()
        self._new = threading.Condition(self._lock)
        self._dropped = 0

    def put(self, item):
        with self._new:
            if self._item is not None:
                self._dropped += 1          # overwriting an unread item
            self._item = item
            self._seq += 1
            self._new.notify_all()

    def get(self, timeout=None):
        """Take the item and leave the slot empty. None if nothing arrived."""
        with self._new:
            if self._item is None:
                self._new.wait(timeout)
            item, self._item = self._item, None
            return item

    def peek(self):
        """Read without consuming. Returns the pair (item, seq)."""
        with self._lock:
            return self._item, self._seq

    @property
    def dropped(self):
        with self._lock:
            return self._dropped


class RateCounter:
    """Rolling frames per second over a fixed window."""

    def __init__(self, window=None):
        self.window = window or cfg.FPS_WINDOW
        self._times = []
        self._lock = threading.Lock()
        self.total = 0

    def tick(self):
        now = time.perf_counter()
        with self._lock:
            self.total += 1
            self._times.append(now)
            if len(self._times) > self.window:
                del self._times[:-self.window]

    def fps(self):
        with self._lock:
            if len(self._times) < 2:
                return 0.0
            span = self._times[-1] - self._times[0]
            return (len(self._times) - 1) / span if span > 0 else 0.0


# Thread 1, vision and AI

class VisionThread(threading.Thread):
    """Owns the camera and detection. Posts results to a one-slot mailbox."""

    def __init__(self, source, depth_provider=None, name="vision"):
        super().__init__(name=name, daemon=True)
        self.source = source
        self.depth_provider = depth_provider
        self.results = LatestOnly()
        self.rate = RateCounter()
        self.frame_ms = []
        self._stop_event = threading.Event()
        self.error = None

    def stop(self):
        self._stop_event.set()

    def run(self):
        try:
            while not self._stop_event.is_set():
                frame, label = self.source.read()
                if frame is None:
                    # A dropped frame is fine on a live camera. We never make
                    # up a command to cover the gap.
                    self._stop_event.wait(0.005)
                    continue

                t0 = time.perf_counter()
                frame = normalize(frame)
                depth = self.depth_provider() if self.depth_provider else None
                det = detect(frame, depth=depth)
                elapsed_ms = (time.perf_counter() - t0) * 1000.0

                self.frame_ms.append(elapsed_ms)
                self.rate.tick()
                self.results.put({
                    "frame": frame, "det": det, "label": label,
                    "ms": elapsed_ms, "t": time.perf_counter(),
                })
        except Exception as exc:                      # keep the reason visible
            self.error = exc
            raise


# Thread 2, user interface

class UIThread(threading.Thread):
    """Draws annotated frames for the main thread to show.

        HighGUI isn't thread safe, so the main thread only calls imshow() and waitKey().
    """

    def __init__(self, vision, name="ui"):
        super().__init__(name=name, daemon=True)
        self.vision = vision
        self.display = LatestOnly()
        self.rate = RateCounter()
        self._stop_event = threading.Event()
        self._last = None
        self.error = None

    def stop(self):
        self._stop_event.set()

    def run(self):
        interval = 1.0 / max(cfg.UI_FPS, 1)
        try:
            while not self._stop_event.is_set():
                started = time.perf_counter()
                item, _ = self.vision.results.peek()
                if item is not None:
                    self._last = item
                if self._last is not None:
                    hud = "vision %.1f fps | ui %.1f fps | dropped %d" % (
                        self.vision.rate.fps(), self.rate.fps(),
                        self.vision.results.dropped)
                    shown = annotate(self._last["frame"], self._last["det"], hud)
                    self.display.put(shown)
                    self.rate.tick()
                # Sleep instead of spinning so the UI does not steal CPU from
                # vision.
                slack = interval - (time.perf_counter() - started)
                if slack > 0:
                    self._stop_event.wait(slack)
        except Exception as exc:
            self.error = exc
            raise


# Entry point

def build_source(kind):
    return ImageSource() if kind == "images" else CameraSource()


def main():
    ap = argparse.ArgumentParser(description="NAVI live vision pipeline")
    ap.add_argument("--source", choices=("camera", "images"), default="camera",
                    help="frame source, defaults to the EMEET webcam")
    ap.add_argument("--headless", action="store_true",
                    help="no GUI window, save annotated frames instead")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after N seconds, 0 means run until q or Ctrl C")
    ap.add_argument("--depth", action="store_true",
                    help="also open the Arducam ToF and fuse its zone verdicts")
    ap.add_argument("--lcd", action="store_true",
                    help="mirror annotated frames to the SPI LCD framebuffer")
    args = ap.parse_args()

    try:
        source = build_source(args.source)
    except RuntimeError as exc:
        print("ERROR: %s" % exc)
        return 1
    print("Source: %s" % source.describe())

    depth_reader = None
    depth_provider = None
    if args.depth:
        import navi_depth
        try:
            depth_reader = navi_depth.DepthReader()
            depth_reader.start()
            depth_provider = depth_reader.latest
            print("Depth : %s" % depth_reader.describe())
        except Exception as exc:
            print("WARNING: depth unavailable (%s), continuing colour only" % exc)

    lcd = LCDWriter() if args.lcd else None
    if lcd:
        print("LCD  : %s %dx%d %dbpp" % (lcd.fb, lcd.w, lcd.h, lcd.bpp))

    vision = VisionThread(source, depth_provider=depth_provider)
    ui = UIThread(vision)
    vision.start()
    ui.start()

    run_dir = None
    if args.headless:
        run_dir = os.path.join(
            cfg.OUTPUT_DIR,
            "run_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
        os.makedirs(run_dir, exist_ok=True)
        print("Headless: annotated frames go to %s" % run_dir)
    else:
        # Fullscreen so it fills the 3.5 inch LCD, which X11 drives.
        cv2.namedWindow("NAVI", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("NAVI", cv2.WND_PROP_FULLSCREEN,
                              cv2.WINDOW_FULLSCREEN)

    print("Running. Press q in the window, or Ctrl C here, to stop.\n")
    started = time.perf_counter()
    saved = 0
    try:
        while True:
            if args.seconds and time.perf_counter() - started >= args.seconds:
                break
            shown = ui.display.get(timeout=0.5)
            if shown is None:
                continue
            if args.headless:
                # About one image per second. Enough to review and will not
                # fill the SD card on a long run.
                if lcd:
                    lcd.write(shown)
                if time.perf_counter() - started >= saved:
                    cv2.imwrite(os.path.join(run_dir, "frame_%04d.jpg" % saved),
                                shown)
                    saved += 1
            else:
                cv2.imshow("NAVI", shown)
                if lcd:
                    lcd.write(shown)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            print("\r vision %.1f fps | ui %.1f fps | frames %d | dropped %d   "
                  % (vision.rate.fps(), ui.rate.fps(),
                     vision.rate.total, vision.results.dropped), end="")
    except KeyboardInterrupt:
        pass
    finally:
        ui.stop()
        vision.stop()
        ui.join(timeout=2.0)
        vision.join(timeout=2.0)
        source.release()
        if depth_reader:
            depth_reader.stop()
        if lcd:
            lcd.close()
        if not args.headless:
            cv2.destroyAllWindows()
        _summary(vision, ui)
    return 0


def _summary(vision, ui):
    print("\n\n===== VISION PIPELINE =====")
    if vision.frame_ms:
        a = np.array(vision.frame_ms)
        print("  frames processed : %d" % len(a))
        print("  per frame ms     : mean %.2f   median %.2f   p95 %.2f   max %.2f"
              % (a.mean(), np.median(a), np.percentile(a, 95), a.max()))
        print("  vision FPS       : mean %.1f   worst %.1f"
              % (1000.0 / a.mean(), 1000.0 / a.max()))
        verdict = "MEETS" if 1000.0 / a.mean() >= cfg.TARGET_FPS else "BELOW"
        print("  %s the %d FPS target"
              % (verdict, cfg.TARGET_FPS))
    print("  ui frames drawn  : %d (%.1f fps)" % (ui.rate.total, ui.rate.fps()))
    print("  frames dropped   : %d (single slot buffer)"
          % vision.results.dropped)
    print("===========================")


if __name__ == "__main__":
    raise SystemExit(main())
