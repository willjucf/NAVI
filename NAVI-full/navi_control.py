"""NAVI control loop: state machine and the UART thread.

    MANUAL   default after init
    AUTO     vision drives
    ESTOP    sends <0,0,1> until reset

The UART thread sends <LEFT,RIGHT,FLAG> at a fixed rate, independent of vision.

    python3 navi_control.py                      camera and UART
    python3 navi_control.py --source images      no camera
    python3 navi_control.py --no-serial          print packets instead
    python3 navi_control.py --link wifi          packets over WiFi (or ble, both)
"""

import argparse
import collections
import csv
import datetime
import os
import sys
import threading
import time

# The UI window and --lcd draw on the Pi's own screen even when started over
# SSH. X11 owns the display, so point at it unless the caller already did.
os.environ.setdefault("DISPLAY", ":0")

import numpy as np

import navi_config as cfg
from navi_vision import (ImageSource, CameraSource, RateCounter,
                         VisionThread, UIThread)


# Finite state machine

MANUAL, AUTO, ESTOP = "MANUAL", "AUTO", "ESTOP"


class FSM:
    """Three states and three events.

        estop latches from any state. reset always lands in MANUAL, never AUTO.
    """

    def __init__(self, start=AUTO):
        self.state = start
        self.history = [start]

    def event(self, ev):
        # ev is one of toggle_mode, estop, reset
        if ev == "estop":
            self.state = ESTOP                       # latch, from any state
        elif ev == "reset":
            if self.state == ESTOP:
                self.state = MANUAL                  # reset always lands safe
        elif ev == "toggle_mode":
            if self.state == MANUAL:
                self.state = AUTO
            elif self.state == AUTO:
                self.state = MANUAL
            # ignored in ESTOP so a stray toggle cannot escape a stop
        if self.history[-1] != self.state:
            self.history.append(self.state)
        return self.state

    def resolve(self, det):
        """Combine the current state with the latest vision result.

        Returns the triple (command, packet, flag).
        """
        if self.state == ESTOP:
            return "E-STOP", "<0,0,1>", 1
        if self.state == MANUAL:
            # MANUAL sends idle until the joystick is read by the ESP32.
            return "MANUAL idle", "<0,0,0>", 0
        if det is None:
            # AUTO with no vision result yet. Do not make one up.
            return "AUTO no frame", "<0,0,1>", 1
        return det["command"], det["packet"], det["flag"]


# Thread 3, UART transmission

def format_packet(left, right, flag):
    """Build <left,right,flag>. Clamps PWM again as the last step before the ESP32."""
    left = int(max(-cfg.PWM_MAX, min(cfg.PWM_MAX, left)))
    right = int(max(-cfg.PWM_MAX, min(cfg.PWM_MAX, right)))
    flag = 1 if flag else 0
    return "<%d,%d,%d>" % (left, right, flag)


STOP_PACKET = "<0,0,1>"


class UartThread(threading.Thread):
    """Sends the current command at a fixed rate.

        Repeats the last command; once it's older than COMMAND_STALE_S, sends a stop.
        port is anything with write() and close().
    """

    def __init__(self, port, source, name="uart"):
        super().__init__(name=name, daemon=True)
        self.port = port
        self.source = source          # callable returning (packet, timestamp)
        self.rate = RateCounter()
        self.sent = []                # recent packets, for the status line
        self.stale_downgrades = 0
        self.errors = 0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def last_sent(self):
        with self._lock:
            return self.sent[-1] if self.sent else None

    def run(self):
        while not self._stop_event.is_set():
            packet, stamped = self.source()
            if packet is None:
                packet = STOP_PACKET
            elif stamped is not None and \
                    time.perf_counter() - stamped > cfg.COMMAND_STALE_S:
                packet = STOP_PACKET
                self.stale_downgrades += 1

            try:
                if self.port is not None:
                    self.port.write(packet.encode("ascii"))
                with self._lock:
                    self.sent.append(packet)
                    if len(self.sent) > 64:
                        del self.sent[:-64]
                self.rate.tick()
            except Exception:
                # Do not kill the thread on a write error. The ESP32 watchdog
                # covers a link that is really gone.
                self.errors += 1

            self._stop_event.wait(cfg.UART_TX_INTERVAL_S)


class NullPort:
    """Stand in for a serial port when running with --no-serial."""

    def __init__(self):
        self.buffer = []

    def write(self, data):
        self.buffer.append(data)
        if len(self.buffer) > 64:
            del self.buffer[:-64]

    def close(self):
        pass


def open_serial(port=None, baud=None):
    """Open the real UART to the ESP32, or raise with a usable message."""
    try:
        import serial  # noqa: F401
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is not installed. Run: sudo apt install python3-serial"
        ) from exc
    from esp_serial import open_esp_serial
    port = port or cfg.SERIAL_PORT
    baud = baud or cfg.SERIAL_BAUD
    try:
        # Reopens until the CP2102 reads clean, see esp_serial.py.
        return open_esp_serial(port, baud, timeout=0, write_timeout=0.1)
    except Exception as exc:
        raise RuntimeError(
            "Could not open %s at %d baud (%s). Check the USB cable to the CP2102 "
            "board, that nothing else (idf.py monitor, esp_link_test.py) has "
            "the port open, and that you are in the dialout group."
            % (port, baud, exc)) from exc


def start_esp_log(link):
    """Print the ESP32's safety-related log lines from the shared UART."""
    def run():
        buf = b""
        while True:
            try:
                chunk = link.read(256)
            except Exception:
                return
            if not chunk:
                time.sleep(0.02)
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if any(k in line for k in ("HALTED", "armed", "state=", "discarded",
                                           "e stop", "ICM-20948",
                                           "ultrasonic")):
                    print("  esp> " + line)
    threading.Thread(target=run, daemon=True, name="esp-log").start()


# Metrics

try:
    import psutil
    _PROC = psutil.Process()
    _PROC.cpu_percent(None)   # first reading is always meaningless, burn it
except Exception:
    psutil = None
    _PROC = None


def read_temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return None


def read_mem_pct():
    try:
        info = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, v = line.split(":")
                info[k] = int(v.strip().split()[0])   # kB
        total = info["MemTotal"]
        avail = info.get("MemAvailable", info.get("MemFree", 0))
        return 100.0 * (total - avail) / total
    except Exception:
        return None


def read_cpu():
    """Per process CPU percent if psutil is present, else the load average."""
    if _PROC is not None:
        try:
            return "cpu:%4.0f%%" % _PROC.cpu_percent(None)
        except Exception:
            pass
    try:
        return "load:%4.2f" % os.getloadavg()[0]
    except (OSError, AttributeError):
        return "load: n/a"


class Metrics:
    """Per-frame timings and state. Each run appends a summary row to a CSV."""

    def __init__(self):
        self.ms = []
        self.recent = collections.deque(maxlen=cfg.FPS_WINDOW)
        self.started = time.time()

    def record(self, ms):
        self.ms.append(ms)
        self.recent.append(ms)

    def fps(self):
        if not self.recent:
            return 0.0
        avg = sum(self.recent) / len(self.recent)
        return 1000.0 / avg if avg > 0 else 0.0

    def summary(self, label="", extra=None):
        if not self.ms:
            print("\nNo frames processed.")
            return None
        a = np.array(self.ms)
        temp, mem = read_temp_c(), read_mem_pct()
        row = {
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "label": label,
            "frames": len(a),
            "mean_ms": round(float(a.mean()), 3),
            "median_ms": round(float(np.median(a)), 3),
            "p95_ms": round(float(np.percentile(a, 95)), 3),
            "max_ms": round(float(a.max()), 3),
            "mean_fps": round(1000.0 / float(a.mean()), 2),
            "worst_fps": round(1000.0 / float(a.max()), 2),
            "temp_c": None if temp is None else round(temp, 1),
            "mem_pct": None if mem is None else round(mem, 1),
        }
        if extra:
            row.update(extra)

        print("\n\n===== BASELINE METRICS =====")
        print("  frames processed : %d" % row["frames"])
        print("  per frame ms     : mean %.2f   median %.2f   p95 %.2f   max %.2f"
              % (row["mean_ms"], row["median_ms"], row["p95_ms"], row["max_ms"]))
        print("  throughput (FPS) : mean %.1f   worst %.1f"
              % (row["mean_fps"], row["worst_fps"]))
        print("  end CPU temp     : %s"
              % ("n/a" if temp is None else "%.1f C" % temp))
        print("  end RAM used     : %s"
              % ("n/a" if mem is None else "%.0f %%" % mem))
        if extra:
            for k, v in extra.items():
                print("  %-17s: %s" % (k, v))
        print("============================")
        self._append_csv(row)
        return row

    def _append_csv(self, row):
        """Append one row so runs can be compared over time."""
        try:
            os.makedirs(os.path.dirname(cfg.METRICS_CSV), exist_ok=True)
            is_new = not os.path.exists(cfg.METRICS_CSV)
            with open(cfg.METRICS_CSV, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(row.keys()))
                if is_new:
                    w.writeheader()
                w.writerow(row)
            print("  metrics appended to %s" % cfg.METRICS_CSV)
        except Exception as exc:
            print("  could not write metrics CSV: %s" % exc)


# Keyboard, non blocking, works over SSH when stdin is a real terminal

class Keyboard:
    KEYMAP = {"m": "toggle_mode", "e": "estop", " ": "estop", "r": "reset"}

    def __init__(self):
        self.enabled = False
        self._fd = None
        self._old = None
        try:
            import termios, tty, select          # POSIX only
            if sys.stdin.isatty():
                self._termios = termios
                self._select = select
                self._fd = sys.stdin.fileno()
                self._old = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
                self.enabled = True
        except Exception:
            self.enabled = False

    def poll(self):
        """Returns an FSM event string, the word quit, or None."""
        if not self.enabled:
            return None
        r, _, _ = self._select.select([sys.stdin], [], [], 0)
        if not r:
            return None
        ch = sys.stdin.read(1)
        if ch in ("q", "\x03"):                   # q or Ctrl C
            return "quit"
        return self.KEYMAP.get(ch)

    def restore(self):
        if self.enabled and self._old is not None:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)


# System initialization

class NaviSystem:
    """Everything the control loop owns. Opens camera, depth, then serial, and reports what came up."""

    def __init__(self, args):
        self.args = args
        self.source = None
        self.depth_reader = None
        self.port = None
        self.vision = None
        self.ui = None
        self.uart = None
        self.fsm = FSM(start=AUTO)
        self.metrics = Metrics()
        self._command = (None, None)
        self._cmd_lock = threading.Lock()

    def init(self):
        print("NAVI initialization")

        if self.args.source == "images":
            self.source = ImageSource()
        else:
            self.source = CameraSource()
        print("  camera : %s" % self.source.describe())

        depth_provider = None
        if not self.args.no_depth:
            import navi_depth
            try:
                self.depth_reader = navi_depth.DepthReader(sim=self.args.sim_depth)
                self.depth_reader.start()
                depth_provider = self.depth_reader.latest
                print("  depth  : %s" % self.depth_reader.describe())
            except Exception as exc:
                print("  depth  : unavailable (%s)" % exc)
        else:
            print("  depth  : disabled by flag")

        link_kind = getattr(self.args, "link", "uart")
        if self.args.no_serial:
            self.port = NullPort()
            print("  uart   : disabled, packets printed only")
        elif link_kind != "uart":
            # Bench testing only: packets over WiFi and/or Bluetooth instead
            # of the UART (esp_wireless.py).
            import esp_wireless
            try:
                self.port = esp_wireless.open_link(link_kind, self.args.esp_ip)
                up = esp_wireless.wait_connected(self.port)
                print("  link   : %s (%s)" % (self.port.describe(),
                                              "connected" if up else "still connecting"))
                if getattr(self.args, "esp_log", False):
                    start_esp_log(self.port)
                    print("  esp log: echoing the ESP32 status lines")
            except RuntimeError as exc:
                print("  link   : %s" % exc)
                print("           falling back to print only")
                self.port = NullPort()
        else:
            try:
                self.port = open_serial(self.args.port, self.args.baud)
                print("  uart   : %s at %d baud" % (self.args.port or cfg.SERIAL_PORT,
                                                    self.args.baud or cfg.SERIAL_BAUD))
                if getattr(self.args, "esp_log", False):
                    start_esp_log(self.port)
                    print("  esp log: echoing the ESP32 status lines")
            except RuntimeError as exc:
                print("  uart   : %s" % exc)
                print("           falling back to print only")
                self.port = NullPort()

        self.vision = VisionThread(self.source, depth_provider=depth_provider)
        self.uart = UartThread(self.port, self.current_command)
        if self.args.window:
            self.ui = UIThread(self.vision)

        print("  fsm    : starting in %s" % self.fsm.state)
        print("  threads: vision, uart%s\n"
              % (", ui" if self.ui else " (ui window disabled)"))
        return self

    def current_command(self):
        """What Thread 3 should transmit right now."""
        with self._cmd_lock:
            return self._command

    def set_command(self, packet):
        with self._cmd_lock:
            self._command = (packet, time.perf_counter())

    def start(self):
        self.vision.start()
        self.uart.start()
        if self.ui:
            self.ui.start()

    def shutdown(self):
        # Order matters. Stop making commands, send a final stop so the ESP32
        # is left halted instead of holding the last drive value, then close.
        for t in (self.ui, self.vision):
            if t:
                t.stop()
        self.uart.stop()
        for t in (self.ui, self.vision, self.uart):
            if t:
                t.join(timeout=2.0)
        try:
            if self.port is not None:
                self.port.write(STOP_PACKET.encode("ascii"))
                self.port.close()
        except Exception:
            pass
        if self.depth_reader:
            self.depth_reader.stop()
        if self.source:
            self.source.release()


# Main loop

def depth_brief(depth):
    """Short 'D 2205C 1916C 1814C' readout of the three ToF zones."""
    if not depth:
        return "D off                "
    parts = []
    for name in ("left", "center", "right"):
        z = depth["zones"][name]
        mm = z["distance_mm"]
        parts.append(("----" if mm is None else "%4d" % min(mm, 9999))
                     + z["verdict"][0])
    return "D " + " ".join(parts)


class DebugLog:
    """One CSV row per vision result (--log), for tracing a decision back to its frame."""

    FIELDS = ["t", "state", "command", "packet", "status", "green_area",
              "red_area", "green_cx", "d_left", "d_center", "d_right",
              "v_left", "v_center", "v_right", "valid_left", "valid_center",
              "valid_right", "depth_reason"]

    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(self.FIELDS)
        self.t0 = time.perf_counter()
        self.snap_dir = path[:-4] + "_snaps"
        os.makedirs(self.snap_dir, exist_ok=True)
        self.last_cat, self.last_snap, self.snaps = None, 0.0, 0

    def write(self, state, command, packet, det):
        depth = det.get("depth") or {}
        zones = depth.get("zones", {})
        row = ["%.3f" % (time.perf_counter() - self.t0), state, command,
               packet, det.get("status", ""), det.get("green_area", 0),
               det.get("red_area", 0), (det.get("centroid") or ("",))[0]]
        for key in ("distance_mm", "verdict", "valid_frac"):
            for name in ("left", "center", "right"):
                v = zones.get(name, {}).get(key, "")
                if key == "valid_frac" and v != "":
                    v = "%.2f" % v
                row.append("" if v is None else v)
        row.append(depth.get("reason", ""))
        self.w.writerow(row)

    def snap(self, item, category):
        """Save an annotated frame when the decision changes. Rate limited."""
        now = time.perf_counter()
        if category == self.last_cat or now - self.last_snap < 0.5:
            return
        self.last_cat, self.last_snap = category, now
        from navi_vision import annotate
        import cv2
        name = "%07.2fs_%s.jpg" % (now - self.t0, category)
        try:
            cv2.imwrite(os.path.join(self.snap_dir, name),
                        annotate(item["frame"], item["det"]))
            self.snaps += 1
        except Exception:
            pass

    def close(self):
        self.fh.close()


def run_loop(sys_, kb, debug_log=None, lcd=None):
    """Feed vision results through the FSM and hand the packet to the UART thread."""
    print("Keys: [m] MANUAL/AUTO   [e] or space E-STOP   [r] reset   [q] quit\n")
    last_print = 0.0
    last_det = None
    last_item = None

    while True:
        ev = kb.poll()
        if ev == "quit":
            break
        elif ev:
            before = sys_.fsm.state
            after = sys_.fsm.event(ev)
            if before != after:
                print("\n  %s -> %s" % (before, after))

        item = sys_.vision.results.peek()[0]
        if item is not None:
            last_det = item["det"]
            sys_.metrics.record(item["ms"])

        command, packet, flag = sys_.fsm.resolve(last_det)
        sys_.set_command(packet)
        if debug_log is not None and item is not None and item is not last_item:
            debug_log.write(sys_.fsm.state, command, packet, item["det"])
            # first word of the reason: RED, GREEN, DEPTH or NO
            debug_log.snap(item, item["det"]["status"].split()[0])
        last_item = item

        now = time.perf_counter()
        if now - last_print >= cfg.PRINT_EVERY:
            last_print = now
            temp = read_temp_c()
            # Show why, not just what. The reason and depth zones
            # make "it stopped for no reason" debuggable.
            reason = last_det["status"] if last_det else "no frame yet"
            sys.stdout.write(
                "\r[%-6s] %-12s %-26s %s  vis:%4.1f tx:%4.1f temp:%s  "
                % (sys_.fsm.state, packet, reason[:26],
                   depth_brief(last_det.get("depth") if last_det else None),
                   sys_.vision.rate.fps(), sys_.uart.rate.fps(),
                   "n/a" if temp is None else "%4.1fC" % temp))
            sys.stdout.flush()
            if lcd and last_item is not None:
                from navi_vision import annotate
                shown = annotate(last_item["frame"], last_item["det"],
                                 "[%s] %s" % (sys_.fsm.state, reason[:30]))
                lcd.write(shown)

        time.sleep(0.002)


def run_scripted(sys_, total=300):
    """Scripted run when stdin isn't a terminal: AUTO, ESTOP, reset, check it stays MANUAL."""
    print("stdin is not a terminal, running a scripted %d frame sequence:" % total)
    print("  AUTO -> estop at 40% -> ESTOP -> reset at 70% -> MANUAL\n")
    last_det = None
    for i in range(total):
        if i == int(total * 0.40):
            sys_.fsm.event("estop")
            print("\n[frame %d] estop triggered" % i)
        if i == int(total * 0.70):
            sys_.fsm.event("reset")
            print("[frame %d] reset, now %s" % (i, sys_.fsm.state))

        item = sys_.vision.results.peek()[0]
        if item is not None:
            last_det = item["det"]
            sys_.metrics.record(item["ms"])

        command, packet, flag = sys_.fsm.resolve(last_det)
        sys_.set_command(packet)

        if i % 60 == 0:
            print("[frame %4d] %-6s  %-22s %s" % (i, sys_.fsm.state,
                                                  command, packet))
        time.sleep(0.005)

    print("\nfinal state: %s   transition history: %s"
          % (sys_.fsm.state, " -> ".join(sys_.fsm.history)))


def main():
    ap = argparse.ArgumentParser(description="NAVI control loop")
    ap.add_argument("--source", choices=("camera", "images"), default="camera")
    ap.add_argument("--port", default=None, help="serial device, default %s"
                    % cfg.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--no-serial", action="store_true",
                    help="do not open a serial port, print packets instead")
    ap.add_argument("--link", choices=("uart", "wifi", "ble", "both"), default="uart",
                    help="bench testing: send packets over WiFi, Bluetooth or both "
                         "instead of the UART")
    ap.add_argument("--esp-ip", default=None,
                    help="ESP32 WiFi address for --link wifi/both (found "
                         "automatically if left out)")
    ap.add_argument("--no-depth", action="store_true",
                    help="skip the Arducam ToF entirely")
    ap.add_argument("--sim-depth", action="store_true",
                    help="use the synthetic depth source instead of the camera")
    ap.add_argument("--window", action="store_true",
                    help="also run the UI thread and show a live window")
    ap.add_argument("--esp-log", action="store_true",
                    help="print the ESP32 status lines coming back over USB")
    ap.add_argument("--scripted", action="store_true",
                    help="force the scripted sequence even on a terminal")
    ap.add_argument("--log", action="store_true",
                    help="write one CSV row per frame to output/debug_<time>.csv")
    ap.add_argument("--lcd", action="store_true",
                    help="mirror annotated frames to the SPI LCD framebuffer")
    args = ap.parse_args()

    try:
        sys_ = NaviSystem(args).init()
    except RuntimeError as exc:
        print("ERROR: %s" % exc)
        return 1

    debug_log = None
    if args.log:
        debug_log = DebugLog(os.path.join(
            cfg.OUTPUT_DIR, "debug_%s.csv"
            % datetime.datetime.now().strftime("%Y%m%d-%H%M%S")))
        print("  log    : %s" % debug_log.path)

    lcd = None
    if args.lcd:
        from navi_vision import LCDWriter
        lcd = LCDWriter()
        print("  LCD    : %s %dx%d" % (lcd.fb, lcd.w, lcd.h))

    kb = Keyboard()
    sys_.start()
    try:
        if kb.enabled and not args.scripted:
            run_loop(sys_, kb, debug_log, lcd=lcd)
        else:
            run_scripted(sys_)
    except KeyboardInterrupt:
        pass
    finally:
        kb.restore()
        if lcd:
            lcd.close()
        if debug_log is not None:
            debug_log.close()
            print("\nDebug log written to %s" % debug_log.path)
            print("%d snapshots in %s" % (debug_log.snaps, debug_log.snap_dir))
        sys_.shutdown()
        sys_.metrics.summary(
            label=args.source,
            extra={
                "packets_sent": sys_.uart.rate.total,
                "stale_downgrades": sys_.uart.stale_downgrades,
                "uart_errors": sys_.uart.errors,
                "final_state": sys_.fsm.state,
            })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
