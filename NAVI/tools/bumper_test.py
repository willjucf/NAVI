"""Ultrasonic bumper check through the Vision PCB's CP2102.

Sends a slow forward command and prints only the ESP32's state lines, so you
can watch the bumper stop and release while moving a hand in front of the
sensor. Expect one HALTED: OBSTACLE when something comes inside 900 mm and one
"armed and running" about 500 ms after it is past 1100 mm again.

No motor on the bench. The command is 50 PWM, and a stop packet is sent on exit.

    python3 tools/bumper_test.py
    python3 tools/bumper_test.py --port /dev/ttyACM0

Close idf.py monitor, esp_link_test.py and navi_control.py first. Only one
program can hold the port.
"""

import argparse
import os
import sys
import time

# Tools sit one level below the modules they import.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import navi_config as cfg

DRIVE_PACKET = b"<50,50,0>"
STOP_PACKET = b"<0,0,1>"
KEEP = (b"state=", b"HALTED", b"armed")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=cfg.SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=cfg.SERIAL_BAUD)
    args = ap.parse_args()

    try:
        import serial  # noqa: F401
    except ImportError:
        sys.exit("pyserial missing: sudo apt install python3-serial")
    from esp_serial import open_esp_serial

    link = open_esp_serial(args.port, args.baud, timeout=0.05)

    print("sending %s on %s. Move a hand within 30 cm of a sensor. Ctrl C to stop"
          % (DRIVE_PACKET.decode(), args.port))
    buf = b""
    try:
        while True:
            link.write(DRIVE_PACKET)
            buf += link.read(512)
            *lines, buf = buf.split(b"\n")
            for line in lines:
                if any(k in line for k in KEEP):
                    print(line.decode(errors="replace").strip())
            time.sleep(0.1)       # 10 Hz, well inside the 500 ms watchdog
    except KeyboardInterrupt:
        pass
    finally:
        link.write(STOP_PACKET)
        link.close()
        print("\nsent stop packet")


if __name__ == "__main__":
    main()
