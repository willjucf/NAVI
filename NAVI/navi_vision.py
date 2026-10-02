"""NAVI vision pipeline. Thread 1 (vision) and Thread 2 (UI).

The detection core (normalize, clean_mask, largest_object, steering_packet,
detect, annotate) is unchanged from SD1. The report cites these by name (6.10,
6.11.1, 6.11.2) and they passed all seven SD1 test cases, so changing them
would void those results.

New here: a live EMEET webcam instead of stills, and the two thread split from
report 6.18 so a slow UI redraw can never stall safety detection.

Run it directly to see the live window:

    python3 navi_vision.py                 live webcam plus UI window
    python3 navi_vision.py --headless      no window (SSH), saves frames to disk
    python3 navi_vision.py --source images cycle test_images/ instead of camera
"""

import argparse
import datetime
import glob
import os
import threading
import time

import cv2
import numpy as np

import navi_config as cfg


# Detection core, unchanged from SD1

def normalize(frame):
    """Shrink big frames to a fixed processing width.

    Lets one minimum area threshold mean the same on a 12 MP phone photo and a
    720p webcam frame (report 6.11.1).
    """
    h, w = frame.shape[:2]
    if w > cfg.PROC_WIDTH:
        scale = cfg.PROC_WIDTH / float(w)
        frame = cv2.resize(frame, (cfg.PROC_WIDTH, int(round(h * scale))),
                           interpolation=cv2.INTER_AREA)
    return frame


def clean_mask(mask):
    """Open then close the mask.

    Open removes speckle. Close fills small holes so a real object is one solid
    blob, not fragments. This is half of what stops glare reading as green,
    since scattered specks never survive to form a blob.
    """
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
    """Turn the green target's horizontal offset into left and right PWM.

    Tank mixing (report 6.10). Equal PWM goes straight. Offset adds to one side
    and subtracts from the other.
    """
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

    The order is the safety critical part (report 6.6, 6.11.2). Red is checked
    before green so a hazard always wins over a path in the same frame. Nothing
    found means STOP, never coasting on the last command.

    depth is the optional zone summary from navi_depth.zone_map(). It can only
    add a stop, never cause motion, so a blocked center forces STOP even with a
    clean green target. Report 6.11.2 planned for this one extra field.
    """
    h, w = frame.shape[:2]
    min_area = cfg.MIN_AREA_FRAC * (w * h)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    green_mask = clean_mask(cv2.inRange(hsv, cfg.LOWER_GREEN, cfg.UPPER_GREEN))
    red_mask = clean_mask(cv2.bitwise_or(
        cv2.inRange(hsv, cfg.LOWER_RED_1, cfg.UPPER_RED_1),
        cv2.inRange(hsv, cfg.LOWER_RED_2, cfg.UPPER_RED_2)))

    green_c, green_area = largest_object(green_mask, min_area)
    red_c, red_area = largest_object(red_mask, min_area)

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
    """Draw the always on overlay from report 6.5.

    Returns a copy so the vision thread can keep using the original while the
    UI thread draws.
    """
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
    """The EMEET USB webcam through V4L2 on the Pi.

    No cv2.CAP_DSHOW on purpose. SD1's webcam.py used it but it is Windows only
    and will not open a camera on Pi OS.
    """

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
        # Shallow driver queue. Capture side half of the single frame buffer
        # in report 6.14.
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
    """Loop test_images/ as a fake stream.

    Keeps the pipeline runnable with no camera and keeps the SD1 baseline
    reproducible on the same input.
    """

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


# Single slot buffer (report 6.14)

class LatestOnly:
    """One slot mailbox that overwrites instead of queuing.

    Report 6.14 wants a buffer holding only the newest frame. A fast producer
    overwrites the pending item instead of growing a list. Memory stays at one
    frame and the consumer always gets fresh data instead of a stale backlog,
    same idea as the no coast rule in detect().
    """

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


# Thread 1, vision and AI (report 6.18)

class VisionThread(threading.Thread):
    """Owns the camera and all detection work.

    Grabs a frame, normalizes, runs detect() and posts to a one slot mailbox.
    Never draws, writes serial or waits on a consumer, so nothing downstream
    can slow detection.
    """

    def __init__(self, source, depth_provider=None, name="vision"):
        super().__init__(name=name, daemon=True)
        self.source = source
        self.depth_provider = depth_provider
        self.results = LatestOnly()
        self.rate = RateCounter()
        self.frame_ms = []
        self._stop = threading.Event()
        self.error = None

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            while not self._stop.is_set():
                frame, label = self.source.read()
                if frame is None:
                    # A dropped frame is fine on a live camera. We never make
                    # up a command to cover the gap.
                    self._stop.wait(0.005)
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


# Thread 2, user interface (report 6.5, 6.18)

class UIThread(threading.Thread):
    """Builds annotated frames for display, separate from detection.

    HighGUI is not thread safe, so this thread does the expensive copy and
    drawing and hands the finished image to the main thread, which only calls
    imshow() and waitKey(). That is the point of report 6.18: a slow display
    cannot hold up driving.

    With no new detection it redraws the last one instead of blocking, so the
    window stays responsive.
    """

    def __init__(self, vision, name="ui"):
        super().__init__(name=name, daemon=True)
        self.vision = vision
        self.display = LatestOnly()
        self.rate = RateCounter()
        self._stop = threading.Event()
        self._last = None
        self.error = None

    def stop(self):
        self._stop.set()

    def run(self):
        interval = 1.0 / max(cfg.UI_FPS, 1)
        try:
            while not self._stop.is_set():
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
                    self._stop.wait(slack)
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
        cv2.namedWindow("NAVI", cv2.WINDOW_NORMAL)

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
                if time.perf_counter() - started >= saved:
                    cv2.imwrite(os.path.join(run_dir, "frame_%04d.jpg" % saved),
                                shown)
                    saved += 1
            else:
                cv2.imshow("NAVI", shown)
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
        print("  %s the %d FPS target (report Table 2.4.1)"
              % (verdict, cfg.TARGET_FPS))
    print("  ui frames drawn  : %d (%.1f fps)" % (ui.rate.total, ui.rate.fps()))
    print("  frames dropped   : %d (single slot buffer, report 6.14)"
          % vision.results.dropped)
    print("===========================")


if __name__ == "__main__":
    raise SystemExit(main())
