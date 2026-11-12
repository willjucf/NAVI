"""NAVI full system on the chair: camera, ToF, state machine and UART to the ESP32,
with touchscreen controls. Starts in STOP (ESTOP), which sends <0,0,1>.

    START     STOP -> MANUAL
    SWITCH    MANUAL <-> AUTO
    SHUTDOWN  tap twice within 3 s: send a stop, close everything and exit

start_navi.sh runs this at login and restarts it after a crash.

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
import sys
import threading
import time

# Draw on the Pi's own screen even when started over SSH.
os.environ.setdefault("DISPLAY", ":0")

import cv2
import numpy as np

import navi_config as cfg
from navi_control import FSM, MANUAL, AUTO, ESTOP, NaviSystem, NullPort, read_temp_c
from navi_vision import annotate


WINDOW = "NAVI"
LOCK_PATH = "/tmp/navi_app.lock"
FONT = cv2.FONT_HERSHEY_SIMPLEX

CONFIRM_S = 3.0          # second SHUTDOWN tap must come within this
DEBOUNCE_S = 0.4         # touch panels can report one tap twice
ESP_SILENT_S = 2.0       # the ESP32 logs twice a second

# BGR
WHITE, GREY, DARK = (255, 255, 255), (150, 150, 150), (30, 30, 30)
RED, GREEN, AMBER, BLUE = (0, 0, 220), (0, 170, 0), (0, 165, 255), (200, 120, 0)

STATE_LABEL = {ESTOP: "STOPPED", MANUAL: "MANUAL", AUTO: "AUTO"}
STATE_COLOR = {ESTOP: RED, MANUAL: BLUE, AUTO: GREEN}


def log(msg):
    print("%s %s" % (datetime.datetime.now().strftime("%H:%M:%S"), msg), flush=True)


# ESP32 status, read back over the same UART

class EspMonitor(threading.Thread):
    """Tracks the ESP32's safety state from its log lines. Logs changes only."""

    STATE_RE = re.compile(r"state=(\S+)")
    USS_RE = re.compile(r"uss=(-?\d+)mm(\(none\))?")
    ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
    EVENTS = ("HALTED", "armed", "e stop", "reset refused", "discarded",
              "NAVI firmware", "ICM-20948", "ultrasonic")

    def __init__(self, link):
        super().__init__(name="esp-log", daemon=True)
        self.link = link
        self.state = None
        self.uss_mm = None
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
            if m.group(1) != self.state:
                log("esp state %s" % m.group(1))
            self.state = m.group(1)
        elif any(k in line for k in self.EVENTS):
            log("esp> " + line)


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
    """Touchscreen front end on the navi_control state machine and threads."""

    KEYS = {ord("m"): "toggle_mode", ord("e"): "estop", ord(" "): "estop",
            ord("r"): "reset"}

    def __init__(self, size):
        self.w, self.h = size
        self.frame_size = (cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT)
        self.layout = Layout(self.w, self.h, *self.frame_size)
        self.sys = None
        self.esp = None
        self.clicks = collections.deque(maxlen=8)
        self.last_tap = 0.0
        self.confirm_until = 0.0
        self.quit = False

    # input

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicks.append((x, y))

    def fire(self, ev, why):
        before = self.sys.fsm.state
        after = self.sys.fsm.event(ev)
        if after != before:
            log("%s -> %s (%s)" % (before, after, why))

    def tap(self, x, y):
        now = time.perf_counter()
        if now - self.last_tap < DEBOUNCE_S:
            return
        self.last_tap = now
        if Layout.hit(self.layout.main, x, y):
            if self.sys.fsm.state == ESTOP:
                self.fire("reset", "START")
            else:
                self.fire("toggle_mode", "SWITCH")
        elif Layout.hit(self.layout.shutdown, x, y):
            if now < self.confirm_until:
                log("SHUTDOWN")
                self.quit = True
            else:
                self.confirm_until = now + CONFIRM_S

    def poll_input(self, wait_ms):
        key = cv2.waitKey(wait_ms) & 0xFF
        if key == ord("q"):
            self.quit = True
        elif key in self.KEYS:
            self.fire(self.KEYS[key], "key %s" % chr(key))
        while self.clicks:
            self.tap(*self.clicks.popleft())

    # screen

    def blank(self, message):
        canvas = np.zeros((self.h, self.w, 3), np.uint8)
        text_in(canvas, message, (0, 0, self.w, self.h), WHITE, 2, 1.0)
        return canvas

    def render(self, item, command, packet, reason):
        L = self.layout
        canvas = np.zeros((self.h, self.w, 3), np.uint8)
        state = self.sys.fsm.state

        x, y, w, h = L.cam
        if item is None:
            text_in(canvas, "starting camera", L.cam, GREY, 1, 0.6)
        else:
            # The overlay shows what is actually sent, not what AUTO would do.
            det = dict(item["det"], command=command, packet=packet)
            shown = annotate(item["frame"], det)
            canvas[y:y + h, x:x + w] = cv2.resize(shown, (w, h),
                                                  interpolation=cv2.INTER_AREA)
            # Outline the view in the state colour so the mode reads at a glance.
            cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1),
                          STATE_COLOR[state], 3)

        x, y, w, h = L.side
        if w >= 30:
            temp = read_temp_c()
            rows = ["%.0f fps" % self.sys.vision.rate.fps(),
                    "%.0f Hz" % self.sys.uart.rate.fps(),
                    "--" if temp is None else "%.0f C" % temp]
            for i, row in enumerate(rows):
                text_in(canvas, row, (x, y + 4 + i * 22, w, 20), GREY, 1, 0.4)

        x, y, w, h = L.bar
        canvas[y:y + h, x:x + w] = DARK
        x, y, w, h = L.info
        line_h = h // 3
        text_in(canvas, "%s  %s" % (STATE_LABEL[state], packet),
                (x, y, w, line_h), STATE_COLOR[state], 2, 0.7, "left")
        text_in(canvas, reason, (x, y + line_h, w, line_h), WHITE, 1, 0.5, "left")
        status, color = self.link_status()
        text_in(canvas, status, (x, y + 2 * line_h, w, line_h), color, 1, 0.5,
                "left")

        if state == ESTOP:
            draw_button(canvas, L.main, "START", GREEN)
        else:
            draw_button(canvas, L.main, "SWITCH", BLUE,
                        "to AUTO" if state == MANUAL else "to MANUAL")
        if time.perf_counter() < self.confirm_until:
            draw_button(canvas, L.shutdown, "TAP AGAIN", RED, "to shut down")
        else:
            draw_button(canvas, L.shutdown, "SHUTDOWN", RED)
        return canvas

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
            esp = "ESP: armed"
        else:
            esp = "ESP: " + self.esp.state
        if self.esp is not None and self.esp.uss_mm is not None:
            esp += " %dmm" % self.esp.uss_mm
        reader = self.sys.depth_reader
        if reader is None or reader.error is not None:
            tof = "ToF OFF"
            bad = True
        else:
            tof = "ToF ok"
        return "%s  %s" % (esp, tof), (RED if bad else GREY)

    # main loop

    def run(self):
        """Same decision step as navi_control.run_loop, plus the screen."""
        last_det = None
        last_item = None
        last_draw = 0.0
        interval = 1.0 / max(cfg.UI_FPS, 1)

        while not self.quit:
            item = self.sys.vision.results.peek()[0]
            if item is not None:
                last_det = item["det"]
                last_item = item
                # The camera may not give the size asked for in navi_config.
                fh, fw = item["frame"].shape[:2]
                if (fw, fh) != self.frame_size:
                    self.frame_size = (fw, fh)
                    self.layout = Layout(self.w, self.h, fw, fh)

            command, packet, flag = self.sys.fsm.resolve(last_det)
            self.sys.set_command(packet)

            now = time.perf_counter()
            if now - last_draw >= interval:
                last_draw = now
                state = self.sys.fsm.state
                if state == ESTOP:
                    reason = "Stopped. Tap START for manual"
                elif state == MANUAL:
                    reason = "Manual: joystick"
                else:
                    reason = last_det["status"] if last_det else "no frame yet"
                cv2.imshow(WINDOW, self.render(last_item, command, packet, reason))
            self.poll_input(5)


def main():
    ap = argparse.ArgumentParser(description="NAVI on the chair")
    ap.add_argument("--window", action="store_true",
                    help="normal window instead of fullscreen")
    ap.add_argument("--screen", default=None,
                    help="screen size WxH, found with xrandr if left out")
    ap.add_argument("--no-serial", action="store_true",
                    help="no ESP32, print packets instead")
    ap.add_argument("--no-depth", action="store_true", help="skip the ToF")
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
    app = App(size)

    def on_signal(signum, frame):
        app.quit = True
    signal.signal(signal.SIGTERM, on_signal)

    if args.window:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    else:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.setMouseCallback(WINDOW, app.on_mouse)
    cv2.imshow(WINDOW, app.blank("Starting NAVI"))
    cv2.waitKey(50)

    log("NAVI starting, screen %dx%d" % size)
    sys_ = NaviSystem(argparse.Namespace(
        source="camera", port=None, baud=None, no_serial=args.no_serial,
        link="uart", esp_ip=None, no_depth=args.no_depth, sim_depth=False,
        window=False, esp_log=False))
    sys_.fsm = FSM(start=ESTOP)          # always boot stopped
    try:
        sys_.init()
    except RuntimeError as exc:
        log("ERROR: %s" % exc)
        cv2.imshow(WINDOW, app.blank("Camera error, retrying"))
        cv2.waitKey(3000)
        return 1
    app.sys = sys_
    if not isinstance(sys_.port, NullPort):
        app.esp = EspMonitor(sys_.port)
        app.esp.start()

    sys_.start()
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        # navi_control's shutdown sends a final stop before closing the port.
        if app.esp:
            app.esp.stop()
        sys_.shutdown()
        cv2.destroyAllWindows()
        log("NAVI stopped in %s" % sys_.fsm.state)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
