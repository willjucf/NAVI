"""ESP32 link test through the Vision PCB's CP2102.

The ESP32 reads packets and logs on the same UART0, so one USB cable does
both: this sends packets, reads back
the firmware's log of what it did, and checks it.

No motor on the bench, so nothing here can drive one. The highest command sent
is 100 PWM.

    python3 tools/esp_link_test.py              run every step, PASS or FAIL each
    python3 tools/esp_link_test.py --monitor    just print the ESP32 log (Ctrl C to quit)
    python3 tools/esp_link_test.py --port /dev/ttyACM0

Close idf.py monitor and navi_control.py first. Only one program can hold the
port.
"""

import argparse
import re
import os
import sys
import threading
import time

# Tools sit one level below the modules they import.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import navi_config as cfg

ANSI = re.compile(r"\x1b\[[0-9;]*m")
STATUS = re.compile(r"state=(\w+) pwm=(-?\d+),(-?\d+) target=(-?\d+),(-?\d+) flag=(\d)")


def open_port(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial missing: sudo apt install python3-serial")
    from esp_serial import open_esp_serial
    try:
        link = open_esp_serial(port, baud, timeout=0.05)
    except Exception as exc:
        sys.exit("could not open %s: %s\n"
                 "is the cable in (ls /dev/ttyUSB* /dev/ttyACM*), is another "
                 "program holding it, are you in the dialout group?" % (port, exc))
    return link


class Reader(threading.Thread):
    """Collects ESP32 log lines with their arrival time."""

    def __init__(self, link, echo=True):
        super().__init__(daemon=True)
        self.link = link
        self.echo = echo
        self.lines = []
        self.lock = threading.Lock()
        self.running = True

    def run(self):
        buf = b""
        while self.running:
            try:
                chunk = self.link.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = ANSI.sub("", raw.decode("ascii", "replace")).strip()
                if not line:
                    continue
                with self.lock:
                    self.lines.append((time.monotonic(), line))
                if self.echo:
                    print("    esp> " + line)

    def since(self, t0):
        with self.lock:
            return [line for t, line in self.lines if t >= t0]


def last_status(lines):
    for line in reversed(lines):
        m = STATUS.search(line)
        if m:
            return {"state": m.group(1), "pwm": (int(m.group(2)), int(m.group(3))),
                    "flag": int(m.group(6))}
    return None


def send_for(link, packet, seconds):
    """Repeat one packet at the real control loop rate."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if packet:
            link.write(packet.encode("ascii"))
        time.sleep(cfg.UART_TX_INTERVAL_S)


results = []


def check(name, ok, detail):
    results.append((name, ok))
    print("  [%s] %s  %s" % ("PASS" if ok else "FAIL", name, detail))


def step(title):
    print("\n== " + title)


def run_all(link, reader):
    stop = "<0,0,1>"

    step("1. Boots halted: listen only, nothing sent")
    t0 = time.monotonic()
    time.sleep(3.0)
    lines = reader.since(t0)
    if not lines:
        check("ESP32 is talking", False,
              "no output in 3 s. Is the NAVI firmware flashed? Try pressing "
              "the EN/RST button on the board, or run with --monitor.")
        return
    s = last_status(lines)
    check("ESP32 is talking", True, "%d lines" % len(lines))
    check("halted with NO_LINK before any packet",
          s is not None and s["state"] in ("NO_LINK", "SIGNAL_LOST") and s["pwm"] == (0, 0),
          s and "state=%s pwm=%s" % (s["state"], s["pwm"]) or "no status line seen")
    if s and s["state"] == "SIGNAL_LOST":
        print("    (SIGNAL_LOST means it heard packets earlier this boot; press EN "
              "to see NO_LINK from a cold start)")

    step("2. Stop packet arms the link but keeps PWM at 0")
    t0 = time.monotonic()
    send_for(link, stop, 2.0)
    s = last_status(reader.since(t0))
    check("link comes up on the first packet", s is not None and s["state"] == "NONE",
          s and "state=%s" % s["state"] or "no status line")
    check("safety flag holds PWM at 0", s is not None and s["pwm"] == (0, 0) and s["flag"] == 1,
          s and "pwm=%s flag=%d" % (s["pwm"], s["flag"]) or "")

    step("3. Forward <100,100,0>, PWM should ramp to 100,100")
    t0 = time.monotonic()
    send_for(link, "<100,100,0>", 2.0)
    s = last_status(reader.since(t0))
    check("forward command reaches the motors", s is not None and s["pwm"] == (100, 100),
          s and "pwm=%s" % (s["pwm"],) or "no status line")

    step("4. Turn <-80,80,0>, sign survives the link")
    t0 = time.monotonic()
    send_for(link, "<-80,80,0>", 2.0)
    s = last_status(reader.since(t0))
    check("negative PWM decoded", s is not None and s["pwm"] == (-80, 80),
          s and "pwm=%s" % (s["pwm"],) or "no status line")

    step("5. Red hazard <150,150,1>, the flag must win over the speed")
    t0 = time.monotonic()
    send_for(link, "<150,150,1>", 1.5)
    s = last_status(reader.since(t0))
    check("flag=1 forces an immediate stop", s is not None and s["pwm"] == (0, 0),
          s and "pwm=%s flag=%d" % (s["pwm"], s["flag"]) or "")

    step("6. Watchdog: drive, then go silent. Must halt inside 500 ms")
    send_for(link, "<100,100,0>", 1.0)
    t_silent = time.monotonic()
    time.sleep(1.5)
    lines = reader.since(t_silent)
    lost = [l for l in lines if "SIGNAL_LOST" in l]
    s = last_status(lines)
    check("SIGNAL_LOST after the Pi goes quiet", bool(lost),
          "seen" if lost else "never reported")
    check("PWM dropped to 0", s is not None and s["pwm"] == (0, 0),
          s and "pwm=%s" % (s["pwm"],) or "")
    print("    note: the halt message is printed on the 10 ms control tick, but its"
          " arrival here is delayed by USB buffering, so this checks the halt"
          " happened, not the exact 500 ms. The 499/500/501 ms boundary is"
          " covered by the virtual firmware suite.")

    step("7. Signal loss is not latched: packets resume, link recovers")
    t0 = time.monotonic()
    send_for(link, stop, 1.5)
    s = last_status(reader.since(t0))
    check("recovers on its own", s is not None and s["state"] == "NONE",
          s and "state=%s" % s["state"] or "")

    step("8. Garbage on the wire is discarded, good packets still land")
    t0 = time.monotonic()
    link.write(b"<abc,,9>junk\x00\xff<1,2,3,4,5,6,7,8,9,10,11>")
    send_for(link, "<60,60,0>", 1.5)
    lines = reader.since(t0)
    s = last_status(lines)
    discarded = any("discarded frame" in l for l in lines)
    check("bad frames are rejected", discarded,
          "firmware logged discards" if discarded else "no discard logged")
    check("good packets after the garbage still work", s is not None and s["pwm"] == (60, 60),
          s and "pwm=%s" % (s["pwm"],) or "")

    step("Finish with the stop packet")
    send_for(link, stop, 0.5)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=cfg.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=cfg.SERIAL_BAUD)
    ap.add_argument("--monitor", action="store_true",
                    help="only print what the ESP32 says")
    ap.add_argument("--quiet", action="store_true",
                    help="hide the raw ESP32 log lines")
    args = ap.parse_args()

    link = open_port(args.port, args.baud)
    reader = Reader(link, echo=not args.quiet)
    reader.start()
    print("listening on %s at %d" % (args.port, args.baud))

    try:
        if args.monitor:
            while True:
                time.sleep(1)
        run_all(link, reader)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            link.write(b"<0,0,1>")
        except Exception:
            pass
        reader.running = False
        time.sleep(0.1)
        link.close()

    if results:
        failed = [n for n, ok in results if not ok]
        print("\n%d passed, %d failed" % (len(results) - len(failed), len(failed)))
        for n in failed:
            print("  FAILED: " + n)
        sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
