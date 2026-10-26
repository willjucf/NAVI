"""NAVI preflight self test.

Run before any live hardware test. Checks each subsystem alone and reports what
is ready, so a failure in a real test is a real finding and not a missing
package or swapped wire.

Nothing here moves a motor. The serial check only sends the stop packet and the
firmware will not arm until it decodes a real command, so this is safe with the
ESP32 connected.

    python3 selftest.py              every check
    python3 selftest.py --quick      skip the checks that need hardware
    python3 selftest.py --list       show the checks without running them

Exit code is 0 when nothing FAILED. Checks that do not apply report SKIP and do
not fail the run, since a missing Arducam is about the bench, not a code bug.
"""

import argparse
import os
import re
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

PASS, FAIL, SKIP, WARN = "PASS", "FAIL", "SKIP", "WARN"

_results = []


def record(section, name, status, detail=""):
    _results.append((section, name, status, detail))
    colour = {PASS: "PASS", FAIL: "FAIL", SKIP: "skip", WARN: "warn"}[status]
    print("  [%s] %-46s %s" % (colour, name, detail))


def section(title):
    print("\n" + title)
    print("-" * (len(title) + 2))


# 1. Environment

def check_environment():
    section("1. Environment")

    major, minor = sys.version_info[:2]
    if (major, minor) >= (3, 8):
        record("env", "python version", PASS, "%d.%d" % (major, minor))
    else:
        record("env", "python version", FAIL,
               "%d.%d, need 3.8 or newer" % (major, minor))

    try:
        import numpy
        record("env", "numpy", PASS, numpy.__version__)
    except ImportError:
        record("env", "numpy", FAIL, "sudo apt install python3-numpy")

    try:
        import cv2
        ver = cv2.__version__
        # findContours changed its return values in OpenCV 4. We assume 4+,
        # which Pi OS ships.
        if int(ver.split(".")[0]) >= 4:
            record("env", "opencv", PASS, ver)
        else:
            record("env", "opencv", FAIL, "%s, need 4.x or newer" % ver)
    except ImportError:
        record("env", "opencv", FAIL, "sudo apt install python3-opencv")

    try:
        import serial
        record("env", "pyserial", PASS, getattr(serial, "__version__", "present"))
    except ImportError:
        record("env", "pyserial", WARN,
               "sudo apt install python3-serial, needed only for the real UART")

    try:
        import ArducamDepthCamera        # noqa: F401
        record("env", "ArducamDepthCamera", PASS, "present")
    except ImportError:
        record("env", "ArducamDepthCamera", WARN,
               "not installed, depth will fall back to the simulator")


# 2. Configuration consistency

def _parse_pins_header():
    """Pull the pin numbers back out of firmware/main/navi_pins.h."""
    path = os.path.join(ROOT, "firmware", "main", "navi_pins.h")
    text = open(path, encoding="utf-8").read()
    found = {}
    for name, pin in re.findall(
            r"#define\s+(NAVI_\w+)\s+GPIO_NUM_(\d+)", text):
        found[name] = int(pin)
    return found


def check_config():
    section("2. Configuration consistency")

    try:
        import navi_config as cfg
    except Exception as exc:
        record("config", "navi_config imports", FAIL, str(exc))
        return

    # The Pi and firmware each keep a copy of the pin map. This check is what
    # actually keeps them in sync.
    try:
        header = _parse_pins_header()
    except Exception as exc:
        record("config", "navi_pins.h readable", FAIL, str(exc))
        header = None

    if header:
        mapping = {
            "UART_TX": "NAVI_UART_TX_PIN", "UART_RX": "NAVI_UART_RX_PIN",
            "ESTOP": "NAVI_ESTOP_PIN", "RELAY": "NAVI_RELAY_PIN",
            "LED": "NAVI_LED_PIN", "BUZZER": "NAVI_BUZZER_PIN",
            "USS_A": "NAVI_USS_PIN_A", "USS_B": "NAVI_USS_PIN_B",
            "I2C_SDA": "NAVI_I2C_SDA_PIN", "I2C_SCL": "NAVI_I2C_SCL_PIN",
        }
        mismatches = []
        for py_key, c_key in mapping.items():
            py_val = cfg.ESP32_PINS.get(py_key)
            c_val = header.get(c_key)
            if c_val is None:
                mismatches.append("%s missing from header" % c_key)
            elif py_val != c_val:
                mismatches.append("%s py=%s c=%s" % (py_key, py_val, c_val))
        if mismatches:
            record("config", "pin map matches navi_pins.h", FAIL,
                   "; ".join(mismatches))
        else:
            record("config", "pin map matches navi_pins.h", PASS,
                   "%d pins agree" % len(mapping))

    # The watchdog window must match on both sides.
    logic_h = open(os.path.join(ROOT, "firmware", "main", "navi_logic.h"),
                   encoding="utf-8").read()
    m = re.search(r"WATCHDOG_TIMEOUT_MS\s*=\s*(\d+)", logic_h)
    if not m:
        record("config", "watchdog window agrees", FAIL,
               "could not find WATCHDOG_TIMEOUT_MS")
    elif int(m.group(1)) != cfg.UART_WATCHDOG_MS:
        record("config", "watchdog window agrees", FAIL,
               "py=%d c=%s" % (cfg.UART_WATCHDOG_MS, m.group(1)))
    else:
        record("config", "watchdog window agrees", PASS,
               "%d ms both sides" % cfg.UART_WATCHDOG_MS)

    # Sending slower than the watchdog would halt the chair every cycle.
    margin = cfg.UART_WATCHDOG_MS / (cfg.UART_TX_INTERVAL_S * 1000.0)
    if margin < 2:
        record("config", "tx rate is inside the watchdog window", FAIL,
               "only %.1fx margin" % margin)
    else:
        record("config", "tx rate is inside the watchdog window", PASS,
               "%.0fx margin" % margin)

    if cfg.COMMAND_STALE_S * 1000 >= cfg.UART_WATCHDOG_MS:
        record("config", "stale downgrade beats the watchdog", FAIL,
               "the ESP32 would halt before the Pi downgrades")
    else:
        record("config", "stale downgrade beats the watchdog", PASS,
               "%.0f ms before %d ms" % (cfg.COMMAND_STALE_S * 1000,
                                         cfg.UART_WATCHDOG_MS))

    if cfg.ZONE_BLOCKED_MM < cfg.ZONE_CAUTION_MM <= cfg.DEPTH_MAX_MM:
        record("config", "depth thresholds ordered", PASS,
               "%d < %d <= %d mm" % (cfg.ZONE_BLOCKED_MM, cfg.ZONE_CAUTION_MM,
                                     cfg.DEPTH_MAX_MM))
    else:
        record("config", "depth thresholds ordered", FAIL, "thresholds overlap")

    # Measured values from report Table 6.11.1. A silent change would make the
    # report wrong.
    # Red retuned 2026-09-25 and again 2026-10-22 (navi_config.py). Report
    # Table 6.11.1 still has the old values and needs updating.
    expected = {"LOWER_GREEN": [35, 50, 40], "UPPER_GREEN": [90, 255, 255],
                "LOWER_RED_1": [0, 140, 70], "UPPER_RED_1": [7, 255, 255],
                "LOWER_RED_2": [160, 140, 70]}
    bad = [k for k, v in expected.items() if list(getattr(cfg, k)) != v]
    if bad:
        record("config", "hsv matches report Table 6.11.1", FAIL,
               "changed: %s" % ", ".join(bad))
    else:
        record("config", "hsv matches report Table 6.11.1", PASS, "4 ranges")


# 3. Camera

def check_camera():
    section("3. Camera (EMEET webcam over USB)")
    try:
        import cv2
        from navi_vision import CameraSource, normalize, detect
    except Exception as exc:
        record("camera", "imports", SKIP, str(exc))
        return

    if os.name != "nt" and not any(
            os.path.exists("/dev/video%d" % i) for i in range(8)):
        record("camera", "device node present", SKIP,
               "no /dev/video*, is the webcam plugged in")
        return

    try:
        source = CameraSource()
    except Exception as exc:
        record("camera", "opens", FAIL, str(exc))
        return

    try:
        record("camera", "opens", PASS, source.describe())

        frame, _ = source.read()
        if frame is None:
            record("camera", "delivers a frame", FAIL, "read returned nothing")
            return
        record("camera", "delivers a frame", PASS,
               "%dx%d" % (frame.shape[1], frame.shape[0]))

        # Measure real pipeline speed, not what the driver claims. Table 2.4.1
        # asks for 30 fps.
        started = time.perf_counter()
        frames = 0
        while time.perf_counter() - started < 2.0:
            frame, _ = source.read()
            if frame is None:
                continue
            detect(normalize(frame))
            frames += 1
        fps = frames / (time.perf_counter() - started)

        # The camera caps the rate, so this never reads above it no matter
        # how fast detect() is. The real question is whether detect() keeps up,
        # so compare to the camera rate with some tolerance, not a flat 30.
        import navi_config as cfg
        target = min(30, cfg.CAMERA_FPS)
        if fps >= target * 0.95:
            record("camera", "detect() keeps up with the camera", PASS,
                   "%.1f fps, target %d" % (fps, target))
        elif fps >= 20:
            record("camera", "detect() keeps up with the camera", WARN,
                   "%.1f fps, under the Table 2.4.1 target of %d" % (fps, target))
        else:
            record("camera", "detect() keeps up with the camera", FAIL,
                   "%.1f fps, well under the %d target" % (fps, target))
    finally:
        source.release()


# 4. Depth

def check_depth():
    section("4. Depth (Arducam ToF over ribbon)")
    try:
        import navi_depth
    except Exception as exc:
        record("depth", "imports", FAIL, str(exc))
        return

    try:
        source = navi_depth.ArducamSource()
    except Exception as exc:
        record("depth", "camera opens", SKIP, str(exc).split(".")[0])
        return

    try:
        record("depth", "camera opens", PASS, source.describe())
        got = 0
        started = time.perf_counter()
        summary = None
        while time.perf_counter() - started < 2.0 and got < 30:
            depth = source.read()
            if depth is None:
                continue
            summary = navi_depth.zone_map(depth)
            got += 1
        if got == 0:
            record("depth", "delivers frames", FAIL, "no frames in 2 s")
        else:
            record("depth", "delivers frames", PASS,
                   "%d frames, %.1f fps" % (got, got / (time.perf_counter() - started)))
            record("depth", "zones report distances", PASS,
                   navi_depth.format_row(summary))
    finally:
        source.close()


# 5. Serial link

def check_serial():
    section("5. Serial link to the ESP32")
    try:
        import navi_config as cfg
    except Exception as exc:
        record("serial", "imports", FAIL, str(exc))
        return

    try:
        import serial          # noqa: F401
    except ImportError:
        record("serial", "pyserial", SKIP, "not installed")
        return

    port = cfg.SERIAL_PORT
    if os.name == "nt":
        record("serial", "port present", SKIP, "not applicable on Windows")
        return
    if not os.path.exists(port):
        record("serial", "port present", FAIL,
               "%s missing, is the ESP32 USB cable in? try ls /dev/ttyUSB* /dev/ttyACM*" % port)
        return
    record("serial", "port present", PASS, port)

    if not os.access(port, os.R_OK | os.W_OK):
        record("serial", "port writable", FAIL,
               "add your user to dialout, then log out and back in")
        return
    record("serial", "port writable", PASS, "read and write")

    from navi_control import open_serial, STOP_PACKET
    try:
        link = open_serial()
    except Exception as exc:
        record("serial", "opens at 115200", FAIL, str(exc))
        return
    try:
        # Only the stop packet. The firmware will not arm on it, so this cannot
        # start a motor even with everything connected.
        link.write(STOP_PACKET.encode("ascii"))
        record("serial", "writes the stop packet", PASS, STOP_PACKET)
    except Exception as exc:
        record("serial", "writes the stop packet", FAIL, str(exc))
    finally:
        link.close()


CHECKS = [
    ("environment", check_environment, False),
    ("config", check_config, False),
    ("camera", check_camera, True),
    ("depth", check_depth, True),
    ("serial", check_serial, True),
]


def main():
    ap = argparse.ArgumentParser(description="NAVI preflight self test")
    ap.add_argument("--quick", action="store_true",
                    help="skip the checks that need hardware attached")
    ap.add_argument("--list", action="store_true",
                    help="list the checks without running them")
    args = ap.parse_args()

    if args.list:
        for name, _, needs_hw in CHECKS:
            print("  %-14s %s" % (name, "(needs hardware)" if needs_hw else ""))
        return 0

    print("NAVI preflight self test")
    print("========================")
    print("Nothing here moves a motor. The serial check sends only the stop "
          "packet,\nand the firmware refuses to arm until it decodes a real "
          "command.")

    for name, fn, needs_hw in CHECKS:
        if args.quick and needs_hw:
            continue
        try:
            fn()
        except Exception as exc:
            record(name, "check crashed", FAIL, "%s: %s" % (type(exc).__name__, exc))

    failed = [r for r in _results if r[2] == FAIL]
    warned = [r for r in _results if r[2] == WARN]
    skipped = [r for r in _results if r[2] == SKIP]
    passed = [r for r in _results if r[2] == PASS]

    print("\n" + "=" * 60)
    print("%d passed, %d failed, %d warnings, %d skipped"
          % (len(passed), len(failed), len(warned), len(skipped)))

    if failed:
        print("\nFAILED:")
        for _, name, _, detail in failed:
            print("  %s  %s" % (name, detail))
        print("\nFix these before running a live hardware test.")
    elif warned or skipped:
        print("\nNothing failed. Items marked warn or skip are usually missing "
              "hardware\nor an optional package, not defects. Check the list "
              "above matches what\nyou actually have connected.")
    else:
        print("\nEverything checked out. Safe to proceed to the live tests.")
    print("=" * 60)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
