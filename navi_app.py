"""NAVI assisted driving, on the chair's LCD.

The joystick drives the chair. NAVI's only output is the 4 relays in the joystick
line: the ESP32 opens them on any hazard and closes them once every hazard is gone.

Cut by the Pi (packet flag, <0,0,1> cut / <0,0,0> clear):
    red in view
    ToF center closer than STOP_DISTANCE_MM, or no valid reading
    camera or ToF silent for SENSOR_STALE_S
    STOP on the screen, until RESUME

Cut by the ESP32:
    E-stop pressed
    ultrasonic closer than BUMPER_STOP_MM
    tilt past 15 degrees
    no packet from the Pi for 500 ms

The app opens OFF. With no packets the ESP32 keeps the relays open.

    START           open the camera, ToF and ESP32 link
    STOP / RESUME   cut the joystick and give it back
    SHUTDOWN        tap twice within 3 s: close everything, back to OFF
    EXIT            (OFF only) tap twice within 3 s: quit to the desktop

    python3 navi_app.py                 fullscreen on the LCD
    python3 navi_app.py --window        normal window
    python3 navi_app.py --no-serial     without the ESP32
"""

import argparse
import collections
import datetime
import fcntl
import os
import re
import signal
import subprocess
import threading
import time

# Draw on the Pi's own screen even when started over SSH.
os.environ.setdefault("DISPLAY", ":0")

import cv2
import numpy as np

import navi_config as cfg
from navi_control import NaviSystem, NullPort, read_temp_c
from navi_vision import annotate


WINDOW = "NAVI"
LOCK_PATH = "/tmp/navi_app.lock"
FONT = cv2.FONT_HERSHEY_SIMPLEX

CUT_PACKET = "<0,0,1>"
CLEAR_PACKET = "<0,0,0>"

CONFIRM_S = 3.0          # second SHUTDOWN tap must come within this
DEBOUNCE_S = 0.4         # touch panels can report one tap twice
ESP_SILENT_S = 2.0       # the ESP32 logs twice a second

# BGR
WHITE, GREY, DARK = (255, 255, 255), (150, 150, 150), (30, 30, 30)
RED, GREEN, AMBER = (0, 0, 220), (0, 170, 0), (0, 140, 255)

# ESP32 halt reasons as the screen shows them. PI_HAZARD is the Pi's own cut,
# which the screen already lists by cause.
ESP_REASONS = {
    "ESTOP_BUTTON": "E-STOP",
    "OBSTACLE": "ULTRASONIC",
    "TILT": "TILT",
    "SIGNAL_LOST": "PI LINK LOST",
    "NO_LINK": "NO PI LINK",
}


def log(msg):
    print("%s %s" % (datetime.datetime.now().strftime("%H:%M:%S"), msg), flush=True)


# ESP32 status, read back over the same UART

class EspMonitor(threading.Thread):
    """Tracks the ESP32's safety state from its log lines. Logs changes only."""

    STATE_RE = re.compile(r"state=(\S+)")
    USS_RE = re.compile(r"uss=(-?\d+)mm(\(none\))?")
    TILT_RE = re.compile(r"tilt=(-?[\d.]+)/(-?[\d.]+)(\(no imu\))?")
    ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
    EVENTS = ("HALTED", "armed", "e stop", "reset refused", "discarded",
              "NAVI firmware", "ICM-20948", "ultrasonic")

    def __init__(self, link):
        super().__init__(name="esp-log", daemon=True)
        self.link = link
        self.state = None
        self.uss_mm = None
        self.tilt = None                # (roll, pitch) degrees, None without the IMU
        self.last_line_at = None
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def silent(self):
        return (self.last_line_at is None
                or time.perf_counter() - self.last_line_at > ESP_SILENT_S)

    def run(self):
        buf = b""
        while not self._stop_event.is_set():
            try:
                chunk = self.link.read(256)
            except Exception:
                return
            if not chunk:
                self._stop_event.wait(0.02)
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._line(self.ANSI_RE.sub("", raw.decode("ascii", "replace")).strip())
            buf = buf[-1024:]               # junk with no newline

    def _line(self, line):
        if not line:
            return
        self.last_line_at = time.perf_counter()
        m = self.STATE_RE.search(line)
        if m:
            u = self.USS_RE.search(line)
            self.uss_mm = int(u.group(1)) if u and not u.group(2) else None
            t = self.TILT_RE.search(line)
            self.tilt = (float(t.group(1)), float(t.group(2))) \
                if t and not t.group(3) else None
            if m.group(1) != self.state:
                log("esp state %s" % m.group(1))
            self.state = m.group(1)
        elif any(k in line for k in self.EVENTS):
            log("esp> " + line)


# What the Pi cuts the relays for

class HazardMonitor:
    """Decides from the camera and the ToF whether the joystick is cut.

        Any hazard cuts at once. All clear needs HAZARD_CLEAR_S with no hazard.
        Starts cut.
    """

    def __init__(self, use_depth=True):
        self.use_depth = use_depth
        self.cut = True
        self.reasons = ["starting"]
        self.clear_since = None
        self._tof_tripped = False

    def update(self, item, now, screen_stop):
        """item is the latest vision result (or None), now is perf_counter()."""
        reasons = []
        if screen_stop:
            reasons.append("STOP button")
        if item is None:
            reasons.append("waiting for camera")
        elif now - item["t"] > cfg.SENSOR_STALE_S:
            reasons.append("camera lost")
        else:
            det = item["det"]
            if det["red_c"] is not None:
                reasons.append("RED")
            if self.use_depth:
                reasons.extend(self._tof(det.get("depth")))

        if reasons:
            self.clear_since = None
            self.cut = True
        else:
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since >= cfg.HAZARD_CLEAR_S:
                self.cut = False
            else:
                # Every hazard is gone; holding for HAZARD_CLEAR_S first.
                reasons.append("clearing")
        self.reasons = reasons
        return self.cut

    def _tof(self, depth):
        # zone_map stamps depth with time.time() in the ToF process.
        if depth is None or time.time() - depth["t"] > cfg.SENSOR_STALE_S:
            self._tof_tripped = True
            return ["ToF lost"]
        mm = depth["zones"]["center"]["distance_mm"]
        if mm is None:
            self._tof_tripped = True
            return ["ToF no reading"]
        limit = cfg.STOP_RELEASE_MM if self._tof_tripped else cfg.STOP_DISTANCE_MM
        if mm < limit:
            self._tof_tripped = True
            return ["ToF %dmm" % mm]
        self._tof_tripped = False
        return []


# Drawing

def screen_size():
    """The X11 screen size, 480x320 for the 3.5 inch LCD if xrandr can't say."""
    try:
        out = subprocess.run(["xrandr", "--current"], capture_output=True,
                             text=True, timeout=3).stdout
        m = re.search(r"current (\d+) x (\d+)", out)
        if m:
            return int(m.group(1)), int(m.group(2))
    except Exception:
        pass
    return 480, 320


def text_in(img, text, rect, color, thick=2, max_scale=1.0, align="center"):
    """Write text as large as fits in rect, up to max_scale."""
    x, y, w, h = rect
    scale = max_scale
    while scale > 0.3:
        (tw, th), _ = cv2.getTextSize(text, FONT, scale, thick)
        if tw <= w - 6 and th <= h - 4:
            break
        scale -= 0.05
    (tw, th), _ = cv2.getTextSize(text, FONT, scale, thick)
    tx = x + (w - tw) // 2 if align == "center" else x + 3
    cv2.putText(img, text, (tx, y + (h + th) // 2), FONT, scale, color, thick,
                cv2.LINE_AA)


class Layout:
    """Camera view on top, control bar along the bottom with the buttons, stats on the right."""

    def __init__(self, w, h, frame_w, frame_h):
        self.w, self.h = w, h
        bar_h = max(h // 4, h - w * frame_h // frame_w)
        cam_h = h - bar_h
        cam_w = min(w, cam_h * frame_w // frame_h)
        pad = max(4, h // 80)
        y, bh = cam_h + pad, bar_h - 2 * pad
        shut_w, main_w = w * 22 // 100, w * 32 // 100
        self.cam = (0, 0, cam_w, cam_h)
        self.side = (cam_w, 0, w - cam_w, cam_h)
        self.bar = (0, cam_h, w, bar_h)
        self.shutdown = (w - pad - shut_w, y, shut_w, bh)
        self.main = (self.shutdown[0] - pad - main_w, y, main_w, bh)
        self.info = (pad, y, self.main[0] - 2 * pad, bh)

    @staticmethod
    def hit(rect, x, y):
        rx, ry, rw, rh = rect
        return rx <= x < rx + rw and ry <= y < ry + rh


def draw_button(img, rect, label, fill, sub=None):
    x, y, w, h = rect
    cv2.rectangle(img, (x, y), (x + w, y + h), fill, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), WHITE, 2)
    if sub:
        text_in(img, label, (x, y, w, h * 62 // 100), WHITE, 2)
        text_in(img, sub, (x, y + h * 55 // 100, w, h * 40 // 100), WHITE, 1, 0.5)
    else:
        text_in(img, label, rect, WHITE, 2)


# The app

class App:
    """Touchscreen front end on the navi_control threads."""

    def __init__(self, size, args):
        self.w, self.h = size
        self.args = args
        self.frame_size = (cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT)
        self.layout = Layout(self.w, self.h, *self.frame_size)
        self.sys = None                 # None while OFF
        self.esp = None
        self.hazards = HazardMonitor(not args.no_depth)
        self.screen_stop = False
        self.clicks = collections.deque(maxlen=8)
        self.last_tap = 0.0
        self.confirm_until = 0.0
        self.want_start = False
        self.want_off = False
        self.off_message = "Tap START to begin"
        self.quit = False

    @property
    def running(self):
        return self.sys is not None

    # starting and stopping the system

    def start_system(self):
        """Open the camera, ToF and ESP32 link. Stays OFF if the camera fails."""
        log("START")
        cv2.imshow(WINDOW, self.blank("Starting NAVI"))
        cv2.waitKey(50)
        a = self.args
        sys_ = NaviSystem(argparse.Namespace(
            source="camera", port=None, baud=None, no_serial=a.no_serial,
            link="uart", esp_ip=None, no_depth=a.no_depth, sim_depth=False,
            window=False, esp_log=False))
        # Cut until the first clean frames, before the UART thread starts.
        sys_.set_command(CUT_PACKET)
        try:
            sys_.init()
        except RuntimeError as exc:
            log("ERROR: %s" % exc)
            self.off_message = "Camera error. Check it and tap START"
            self._close(sys_, None)
            return
        # Fresh state for every run.
        self.hazards = HazardMonitor(not a.no_depth)
        self.screen_stop = False
        self.frame_size = (cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT)
        self.layout = Layout(self.w, self.h, *self.frame_size)
        self.esp = None
        if not isinstance(sys_.port, NullPort):
            self.esp = EspMonitor(sys_.port)
            self.esp.start()
        sys_.start()
        self.sys = sys_
        self.off_message = "Tap START to begin"

    def stop_system(self):
        """Close everything and go back to OFF. The final packet is a cut."""
        if not self.running:
            return
        log("SHUTDOWN, system off")
        cv2.imshow(WINDOW, self.blank("Shutting down"))
        cv2.waitKey(50)
        self._close(self.sys, self.esp)
        self.sys = None
        self.esp = None

    @staticmethod
    def _close(sys_, esp):
        if esp:
            esp.stop()
        try:
            # navi_control's shutdown sends a final <0,0,1>, so the relays open.
            if sys_.vision is not None:
                sys_.shutdown()
            else:
                # init failed part way: release whatever it opened.
                if sys_.depth_reader:
                    sys_.depth_reader.stop()
                if sys_.port is not None:
                    sys_.port.close()
                if sys_.source:
                    sys_.source.release()
        except Exception as exc:
            log("shutdown error: %s" % exc)

    # input

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicks.append((x, y))

    def toggle_stop(self, why):
        if not self.running:
            return
        self.screen_stop = not self.screen_stop
        log("%s (%s)" % ("screen STOP" if self.screen_stop else "screen RESUME", why))

    def tap(self, x, y):
        now = time.perf_counter()
        if now - self.last_tap < DEBOUNCE_S:
            return
        self.last_tap = now
        if Layout.hit(self.layout.main, x, y):
            self.confirm_until = 0.0
            if self.running:
                self.toggle_stop("button")
            else:
                self.want_start = True
        elif Layout.hit(self.layout.shutdown, x, y):
            if now < self.confirm_until:
                self.confirm_until = 0.0
                if self.running:
                    self.want_off = True
                else:
                    log("EXIT")
                    self.quit = True
            else:
                self.confirm_until = now + CONFIRM_S

    def poll_input(self, wait_ms):
        key = cv2.waitKey(wait_ms) & 0xFF
        if key == ord("q"):
            self.quit = True
        elif key in (ord("s"), ord(" ")):
            self.toggle_stop("key")
        while self.clicks:
            self.tap(*self.clicks.popleft())

    # status

    def esp_reason(self):
        """The ESP32's own halt reason for the screen, or None."""
        if self.esp is None or self.esp.silent():
            return None
        return ESP_REASONS.get(self.esp.state)

    def stop_reasons(self):
        """Everything holding the relays open right now, Pi side and ESP side."""
        reasons = list(self.hazards.reasons)
        esp = self.esp_reason()
        if esp:
            reasons.insert(0, esp)
        if not reasons and self.hazards.cut:
            reasons = ["clearing"]
        return reasons

    def link_status(self):
        """One line on the ESP32 link and the ToF, red when something's off."""
        bad = False
        if self.esp is None:
            esp = "ESP: NO LINK"
            bad = True
        elif self.esp.silent():
            esp = "ESP: silent"
            bad = True
        elif self.esp.state in (None, "NONE"):
            esp = "ESP: relays on"
        else:
            esp = "ESP: " + self.esp.state
        reader = self.sys.depth_reader
        if not self.hazards.use_depth:
            tof = "ToF off (flag)"
        elif reader is None or reader.error is not None:
            tof = "ToF OFF"
            bad = True
        else:
            tof = "ToF ok"
        return "%s  %s" % (esp, tof), (RED if bad else GREY)

    def side_rows(self, item):
        """Rates, temperature and the live sensor readings for the side column."""
        temp = read_temp_c()
        tof = None
        if item is not None and item["det"].get("depth"):
            tof = item["det"]["depth"]["zones"]["center"]["distance_mm"]
        esp_live = self.esp is not None and not self.esp.silent()
        uss = self.esp.uss_mm if esp_live else None
        tilt = self.esp.tilt if esp_live else None

        def value(v, fmt):
            return (fmt % v, WHITE) if v is not None else ("--", GREY)

        return [
            ("%.0f fps" % self.sys.vision.rate.fps(), GREY),
            ("%.0f Hz" % self.sys.uart.rate.fps(), GREY),
            ("--" if temp is None else "%.0f C" % temp, GREY),
            ("ToF mm", GREY), value(tof, "%d"),
            ("USS mm", GREY), value(uss, "%d"),
            ("IMU deg", GREY),
            value(None if tilt is None else tilt[0], "R %.1f"),
            value(None if tilt is None else tilt[1], "P %.1f"),
        ]

    # screen

    def blank(self, message):
        canvas = np.zeros((self.h, self.w, 3), np.uint8)
        text_in(canvas, message, (0, 0, self.w, self.h), WHITE, 2, 1.0)
        return canvas

    def render_off(self):
        """The OFF screen: nothing running, START and EXIT."""
        L = self.layout
        canvas = np.zeros((self.h, self.w, 3), np.uint8)
        x, y, w, h = 0, 0, self.w, L.bar[1]
        text_in(canvas, "NAVI", (x, y + h // 6, w, h * 2 // 5), WHITE, 3, 2.0)
        text_in(canvas, "assisted driving", (x, y + h * 3 // 5, w, h // 6),
                GREY, 1, 0.7)

        x, y, w, h = L.bar
        canvas[y:y + h, x:x + w] = DARK
        x, y, w, h = L.info
        line_h = h // 3
        text_in(canvas, "OFF", (x, y, w, line_h), GREY, 2, 0.7, "left")
        text_in(canvas, "joystick cut", (x, y + line_h, w, line_h), WHITE, 1,
                0.5, "left")
        text_in(canvas, self.off_message, (x, y + 2 * line_h, w, line_h), GREY,
                1, 0.5, "left")

        draw_button(canvas, L.main, "START", GREEN)
        if time.perf_counter() < self.confirm_until:
            draw_button(canvas, L.shutdown, "TAP AGAIN", RED, "to exit")
        else:
            draw_button(canvas, L.shutdown, "EXIT", RED)
        return canvas

    def render(self, item, packet, driving, reasons):
        L = self.layout
        canvas = np.zeros((self.h, self.w, 3), np.uint8)
        color = GREEN if driving else RED

        x, y, w, h = L.cam
        if item is None:
            text_in(canvas, "starting camera", L.cam, GREY, 1, 0.6)
        else:
            det = item["det"]
            if det["red_c"] is not None:
                seen = "RED HAZARD"
            elif det["green_c"] is not None:
                seen = "GREEN TARGET (display only)"
            else:
                seen = "no target"
            # The overlay shows what is sent, not the old AUTO steering.
            det = dict(det, command="DRIVE" if driving else "STOP",
                       packet=packet, status=seen, color=color)
            shown = annotate(item["frame"], det)
            canvas[y:y + h, x:x + w] = cv2.resize(shown, (w, h),
                                                  interpolation=cv2.INTER_AREA)
            # Outline the view in the state colour so it reads at a glance.
            cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), color, 3)

        x, y, w, h = L.side
        if w >= 30:
            for i, (row, rcolor) in enumerate(self.side_rows(item)):
                text_in(canvas, row, (x, y + 2 + i * 23, w, 21), rcolor, 1, 0.4)

        x, y, w, h = L.bar
        canvas[y:y + h, x:x + w] = DARK
        x, y, w, h = L.info
        line_h = h // 3
        text_in(canvas, "DRIVE" if driving else "STOPPED",
                (x, y, w, line_h), color, 2, 0.7, "left")
        text_in(canvas, "joystick live" if driving else ", ".join(reasons),
                (x, y + line_h, w, line_h), WHITE, 1, 0.5, "left")
        status, scolor = self.link_status()
        text_in(canvas, status, (x, y + 2 * line_h, w, line_h), scolor, 1, 0.5,
                "left")

        if self.screen_stop:
            draw_button(canvas, L.main, "RESUME", GREEN)
        else:
            draw_button(canvas, L.main, "STOP", AMBER)
        if time.perf_counter() < self.confirm_until:
            draw_button(canvas, L.shutdown, "TAP AGAIN", RED, "to shut down")
        else:
            draw_button(canvas, L.shutdown, "SHUTDOWN", RED)
        return canvas

    # main loop

    def run(self):
        """OFF until START, then NAVI until SHUTDOWN, until EXIT."""
        interval = 1.0 / max(cfg.UI_FPS, 1)
        try:
            while not self.quit:
                if self.want_start:
                    self.want_start = False
                    self.start_system()
                if not self.running:
                    cv2.imshow(WINDOW, self.render_off())
                    self.poll_input(int(interval * 1000))
                    continue
                self.run_system(interval)
        finally:
            self.stop_system()

    def run_system(self, interval):
        """One run of NAVI, from START until SHUTDOWN or EXIT."""
        last_item = None
        last_draw = 0.0
        last_cut = None
        last_reasons = None

        while not self.quit and not self.want_off:
            item = self.sys.vision.results.peek()[0]
            if item is not None:
                last_item = item
                # The camera may not give the size asked for in navi_config.
                fh, fw = item["frame"].shape[:2]
                if (fw, fh) != self.frame_size:
                    self.frame_size = (fw, fh)
                    self.layout = Layout(self.w, self.h, fw, fh)

            now = time.perf_counter()
            cut = self.hazards.update(last_item, now, self.screen_stop)
            packet = CUT_PACKET if cut else CLEAR_PACKET
            self.sys.set_command(packet)
            # Log every change of state or of why it is cut, so the log
            # always shows the current reason, not just the first one.
            reasons_now = self.hazards.reasons if cut else []
            if cut != last_cut or reasons_now != last_reasons:
                log("relays %s%s" % ("CUT: " if cut else "allowed",
                                     ", ".join(reasons_now)))
                last_cut = cut
                last_reasons = list(reasons_now)

            if now - last_draw >= interval:
                last_draw = now
                reasons = self.stop_reasons()
                driving = not cut and self.esp_reason() is None and \
                    self.esp is not None and not self.esp.silent()
                cv2.imshow(WINDOW, self.render(last_item, packet, driving, reasons))
            self.poll_input(5)

        self.want_off = False
        self.stop_system()


def main():
    ap = argparse.ArgumentParser(description="NAVI assisted driving")
    ap.add_argument("--window", action="store_true",
                    help="normal window instead of fullscreen")
    ap.add_argument("--screen", default=None,
                    help="screen size WxH, found with xrandr if left out")
    ap.add_argument("--no-serial", action="store_true",
                    help="no ESP32, print packets instead")
    ap.add_argument("--no-depth", action="store_true",
                    help="skip the ToF. It then never cuts the relays")
    args = ap.parse_args()

    # One copy only. A second would fight the first for the camera and UART.
    lock = open(LOCK_PATH, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("NAVI is already running. Stop it with: pkill -f navi_app.py")
        return 0

    size = (tuple(int(v) for v in args.screen.lower().split("x"))
            if args.screen else screen_size())
    app = App(size, args)

    def on_signal(signum, frame):
        app.quit = True
    signal.signal(signal.SIGTERM, on_signal)

    if args.window:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    else:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.setMouseCallback(WINDOW, app.on_mouse)

    log("NAVI assist open (OFF), screen %dx%d, stop distance %d mm"
        % (size + (cfg.STOP_DISTANCE_MM,)))
    try:
        app.run()           # closes the system on the way out
    except KeyboardInterrupt:
        app.stop_system()
    finally:
        cv2.destroyAllWindows()
        log("NAVI closed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
