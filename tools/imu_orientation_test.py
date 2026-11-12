"""IMU baselines: roll and pitch at rest in each orientation.

For each orientation, place the board, press EN and keep it still; the up axis is
picked at boot.

    python3 tools/imu_orientation_test.py --port /dev/ttyUSB0
    python3 tools/imu_orientation_test.py --ip 172.20.10.4
    python3 tools/imu_orientation_test.py --ble

Results go to output/imu_orientation_<time>.csv.
"""

import argparse
import csv
import datetime
import math
import os
import re
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
NAVI_DIR = os.path.dirname(HERE)
sys.path.insert(0, NAVI_DIR)
sys.path.insert(0, HERE)

from navi_console import BleTransport, SerialTransport, TcpTransport, default_port

TILT_LIMIT_DEG = 15.0          # TILT_LIMIT_DEG in the firmware
SETTLE_S = 2.0                 # after the first tilt reading
SAMPLE_S = 10.0
REBOOT_TIMEOUT_S = 60.0

ORIENTATIONS = [
    "flat, chip side up",
    "flat, chip side down",
    "upright, edge 1 down",
    "upright, edge 2 down (edge 1 rotated 90 deg)",
    "upright, edge 3 down (opposite edge 1)",
    "upright, edge 4 down (opposite edge 2)",
]

STAMP = re.compile(r"^[IWE] \((\d+)\) ")
TILT = re.compile(r"tilt=(-?[\d.]+)/(-?[\d.]+)(\(no imu\))?")
UP_AXIS = re.compile(r"IMU up axis (\S+), board ([\d.]+) deg off square")


def drain(transport):
    lines = []
    while not transport.lines.empty():
        lines.append(transport.lines.get_nowait())
    return lines


def wait_for_reboot(transport):
    """Block until the ESP has rebooted and is sending tilt again.

    A reboot shows as the uptime stamp going backwards or, over WiFi and BLE,
    as the link dropping and coming back. Returns (up_axis, off_square)."""
    last_stamp = None
    dropped = False
    rebooted = False
    up, off = "?", None
    start = time.time()
    while time.time() - start < REBOOT_TIMEOUT_S:
        if not transport.connected:
            dropped = True
        for line in drain(transport):
            m = UP_AXIS.search(line)
            if m:
                up, off = m.group(1), float(m.group(2))
                rebooted = True
            m = STAMP.match(line)
            if m:
                stamp = int(m.group(1))
                if last_stamp is not None and stamp < last_stamp:
                    rebooted = True
                last_stamp = stamp
            if dropped and transport.connected:
                rebooted = True
            if rebooted and TILT.search(line) and "(no imu)" not in line:
                return up, off
        time.sleep(0.05)
    return None


def sample(transport, seconds):
    rolls, pitches, no_imu = [], [], 0
    drain(transport)
    end = time.time() + seconds
    while time.time() < end:
        for line in drain(transport):
            m = TILT.search(line)
            if not m:
                continue
            if m.group(3):
                no_imu += 1
            else:
                rolls.append(float(m.group(1)))
                pitches.append(float(m.group(2)))
        left = end - time.time()
        print("\r  recording... %4.1f s left, %d readings " % (max(left, 0), len(rolls)),
              end="", flush=True)
        time.sleep(0.1)
    print()
    return rolls, pitches, no_imu


def off_square(roll, pitch):
    # Angle between gravity and the up axis, from the firmware's roll/pitch.
    c = math.cos(math.radians(roll)) * math.cos(math.radians(pitch))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def summarize(name, note, up, boot_off, rolls, pitches):
    r_mean, p_mean = statistics.mean(rolls), statistics.mean(pitches)
    worst = max(abs(r_mean), abs(p_mean))
    return {
        "orientation": name,
        "note": note,
        "up_axis": up,
        "boot_off_square_deg": "" if boot_off is None else "%.1f" % boot_off,
        "readings": len(rolls),
        "roll_mean": "%.2f" % r_mean,
        "roll_sd": "%.2f" % statistics.pstdev(rolls),
        "roll_min": "%.1f" % min(rolls),
        "roll_max": "%.1f" % max(rolls),
        "pitch_mean": "%.2f" % p_mean,
        "pitch_sd": "%.2f" % statistics.pstdev(pitches),
        "pitch_min": "%.1f" % min(pitches),
        "pitch_max": "%.1f" % max(pitches),
        "off_square_deg": "%.1f" % off_square(r_mean, p_mean),
        "margin_to_limit_deg": "%.1f" % (TILT_LIMIT_DEG - worst),
    }


def print_table(rows):
    print("\n%-46s %4s %14s %14s %6s %7s" % ("orientation", "up", "roll (sd)",
                                             "pitch (sd)", "off", "margin"))
    for r in rows:
        print("%-46s %4s %8s (%s) %8s (%s) %6s %7s" % (
            r["orientation"][:46], r["up_axis"], r["roll_mean"], r["roll_sd"],
            r["pitch_mean"], r["pitch_sd"], r["off_square_deg"], r["margin_to_limit_deg"]))
    print("off = degrees from the up axis. margin = degrees left before the %.0f deg "
          "trip on the worse of roll/pitch." % TILT_LIMIT_DEG)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    how = ap.add_mutually_exclusive_group()
    how.add_argument("--port", help="serial port (default: the CP2102)")
    how.add_argument("--ip", help="ESP's WiFi address")
    how.add_argument("--ble", action="store_true", help="connect over Bluetooth")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tcp-port", type=int, default=3333)
    ap.add_argument("--seconds", type=float, default=SAMPLE_S, help="recording time per orientation")
    args = ap.parse_args()

    if args.ip:
        transport = TcpTransport(args.ip, args.tcp_port)
    elif args.ble:
        transport = BleTransport()
    else:
        transport = SerialTransport(args.port or default_port(), args.baud)

    print("Connecting to %s..." % transport.describe())
    start = time.time()
    while not transport.connected and time.time() - start < 30:
        time.sleep(0.2)
    if not transport.connected:
        sys.exit("Could not connect. Is another program holding the link?")
    print("Connected.\n")
    print("Before you start, mark edges 1-4 on the board (tape or marker), so the")
    print("upright runs can be repeated and matched to the final mounting.\n")

    out_dir = os.path.join(NAVI_DIR, "output")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "imu_orientation_%s.csv"
                            % datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    rows = []
    queue = list(ORIENTATIONS)

    try:
        while True:
            if queue:
                name = queue.pop(0)
                answer = input("Next: %s. Enter to run, s to skip, q to finish: "
                               % name).strip().lower()
                if answer == "q":
                    break
                if answer == "s":
                    continue
            else:
                name = input("Extra orientation name (e.g. final mount), "
                             "or Enter to finish: ").strip()
                if not name:
                    break
            note = input("  Note (which edge is down, what it rests on), optional: ").strip()

            while True:
                print("  Place the board: %s. Keep it still, then press EN/RESET on the board."
                      % name)
                result = wait_for_reboot(transport)
                if result is None:
                    print("  No reboot seen in %.0f s (or the IMU didn't come up)." % REBOOT_TIMEOUT_S)
                    if input("  Try again? [Y/n] ").strip().lower() == "n":
                        break
                    continue
                up, boot_off = result
                print("  Rebooted%s. Settling..." % ("" if up == "?" else
                                                     ", up axis %s, %.1f deg off square" % (up, boot_off)))
                time.sleep(SETTLE_S)
                rolls, pitches, no_imu = sample(transport, args.seconds)
                if not rolls:
                    print("  No tilt readings%s." % (" (IMU reported missing)" if no_imu else ""))
                    if input("  Try again? [Y/n] ").strip().lower() == "n":
                        break
                    continue
                row = summarize(name, note, up, boot_off, rolls, pitches)
                print("  roll %s (sd %s, %s..%s)  pitch %s (sd %s, %s..%s)  off %s deg" % (
                    row["roll_mean"], row["roll_sd"], row["roll_min"], row["roll_max"],
                    row["pitch_mean"], row["pitch_sd"], row["pitch_min"], row["pitch_max"],
                    row["off_square_deg"]))
                if float(row["roll_sd"]) > 0.5 or float(row["pitch_sd"]) > 0.5:
                    print("  Noisy (sd over 0.5 deg). Was the board moving?")
                if input("  Keep this run? [Y/n/r=redo] ").strip().lower() in ("n", "r"):
                    continue
                rows.append(row)
                with open(out_path, "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=list(row.keys()))
                    w.writeheader()
                    w.writerows(rows)
                break
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        transport.close()

    if rows:
        print_table(rows)
        print("\nSaved: %s" % out_path)
    else:
        print("No runs recorded.")


if __name__ == "__main__":
    main()
