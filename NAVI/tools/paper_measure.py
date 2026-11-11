"""Guided measurements for the conference paper tables.

Walks through each test step, measures for a few seconds, and saves one CSV
row per step to output/paper_<mode>_<time>.csv. Press Enter to measure, s to
skip a step. After each step, Enter keeps the result and r measures it again.

    python3 tools/paper_measure.py vision   webcam detection and timing (Table II)
    python3 tools/paper_measure.py uss      ultrasonic through the ESP32 (Table III)
    python3 tools/paper_measure.py tof      ToF center zone (Table IV)

vision also saves the annotated frame from each step as a JPG next to the CSV,
for the paper figure. uss sends a slow <50,50,0> drive command so the ESP32
reports the bumper state as well as the distance. Close navi_control.py and
anything else holding the camera or serial port first.
"""

import argparse
import collections
import csv
import os
import re
import sys
import time

import numpy as np

# Tools sit one level below the modules they import.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import navi_config as cfg

VISION_STEPS = [
    "both markers, red left / green right, about 1 m (39 in) away",
    "both markers, red right / green left, about 1 m (39 in) away",
    "both markers, side by side in the center, about 1 m (39 in) away",
    "both markers, about 0.5 m (20 in) away",
    "both markers, about 1.5 m (59 in) away",
    "both markers, about 2 m (79 in) away",
    "both markers, red high / green low",
    "both markers, tilted about 45 degrees",
    "both markers, room lights dimmed or off (window light only)",
    "both markers, held by hand and moving slowly",
    "FIGURE: green marker only, right of center (steer right)",
    "FIGURE: red marker only, center (red hazard stop)",
    "nothing red or green in view (false detection check)",
]
VISION_FRAMES = 90             # about 3 s at 30 fps

USS_CM = [30, 50, 75, 90, 100, 110, 120, 150, 200]
USS_SECONDS = 8                # the ESP32 reports twice a second
DRIVE_PACKET = b"<50,50,0>"
STOP_PACKET = b"<0,0,1>"

TOF_CM = [50, 75, 90, 110, 150, 190, 210, 300]
TOF_FRAMES = 30


def ask(prompt):
    """Enter to measure, s to skip."""
    return input("\n%s\n  Enter to measure, s to skip: " % prompt).strip().lower() != "s"


def keep():
    return input("  Enter to keep, r to redo: ").strip().lower() != "r"


def stats(values):
    a = np.asarray(values, dtype=float)
    if a.size == 0:
        return {"n": 0, "mean": "", "std": "", "min": "", "max": ""}
    return {"n": int(a.size), "mean": round(float(a.mean()), 2),
            "std": round(float(a.std()), 2),
            "min": round(float(a.min()), 2), "max": round(float(a.max()), 2)}


def majority(values):
    return collections.Counter(values).most_common(1)[0][0] if values else ""


def cm_to_in(cm):
    return "" if cm == "" else round(cm / 2.54, 1)


def error_pct(measured_cm, actual_cm):
    if measured_cm == "":
        return ""
    return round(abs(measured_cm - actual_cm) / actual_cm * 100.0, 2)


# Vision

def measure_vision(cam, out_dir, step_no):
    import cv2
    from navi_vision import normalize, detect, annotate

    ms, red, green = [], 0, 0
    last = None
    t_start = time.perf_counter()
    frames = 0
    while frames < VISION_FRAMES:
        frame, _ = cam.read()
        if frame is None:
            continue
        frames += 1
        t0 = time.perf_counter()
        frame = normalize(frame)
        det = detect(frame)
        ms.append((time.perf_counter() - t0) * 1000.0)
        red += det["red_c"] is not None
        green += det["green_c"] is not None
        last = (frame, det)
    cam_fps = frames / (time.perf_counter() - t_start)

    shot = os.path.join(out_dir, "step%02d.jpg" % step_no)
    cv2.imwrite(shot, annotate(*last))
    s = stats(ms)
    return {
        "frames": frames,
        "red_pct": round(100.0 * red / frames, 1),
        "green_pct": round(100.0 * green / frames, 1),
        "mean_ms": s["mean"], "std_ms": s["std"],
        "p95_ms": round(float(np.percentile(ms, 95)), 2), "max_ms": s["max"],
        "camera_fps": round(cam_fps, 1),
        "image": os.path.basename(shot),
    }


def run_vision(writer, out_dir):
    from navi_vision import CameraSource
    cam = CameraSource()
    print("camera: %s" % cam.describe())
    for _ in range(10):                    # let auto exposure settle
        cam.read()
    try:
        for i, step in enumerate(VISION_STEPS, 1):
            if not ask("STEP %d/%d: %s" % (i, len(VISION_STEPS), step)):
                continue
            while True:
                r = measure_vision(cam, out_dir, i)
                print("  red seen %s%%  green seen %s%%  %s ms avg (p95 %s)  camera %s fps  -> %s"
                      % (r["red_pct"], r["green_pct"], r["mean_ms"], r["p95_ms"],
                         r["camera_fps"], r["image"]))
                if keep():
                    break
            writer.writerow(dict(step=i, setup=step, **r))
    finally:
        cam.release()


# Ultrasonic

USS_RE = re.compile(rb"state=(\w+).*?uss=(-?\d+)mm(\(none\))?")


def measure_uss(link):
    link.reset_input_buffer()
    readings, misses, states = [], 0, []
    buf = b""
    t_end = time.time() + USS_SECONDS
    while time.time() < t_end:
        link.write(DRIVE_PACKET)
        buf += link.read(512)
        *lines, buf = buf.split(b"\n")
        for line in lines:
            m = USS_RE.search(line)
            if not m:
                continue
            states.append(m.group(1).decode())
            if m.group(3) or int(m.group(2)) < 0:
                misses += 1
            else:
                readings.append(int(m.group(2)) / 10.0)
        time.sleep(0.1)       # 10 Hz, well inside the 500 ms watchdog
    return readings, misses, states


def run_uss(writer, port):
    from esp_serial import open_esp_serial
    link = open_esp_serial(port, timeout=0.05)
    print("ESP32 on %s. Aim the sensor straight at a flat target (box side, "
          "book, wall). Measure from the sensor face." % port)
    try:
        for cm in USS_CM:
            if not ask("Place the target at %d cm (%.1f in)" % (cm, cm_to_in(cm))):
                continue
            while True:
                readings, misses, states = measure_uss(link)
                s = stats(readings)
                bumper = majority(states)
                if not states:
                    print("  no status lines from the ESP32. It may be stuck in reset "
                          "or boot mode: press EN on the dev kit, wait 2 s, press r")
                print("  %d readings, %d no echo   mean %s cm / %s in  (%s to %s cm)   state %s"
                      % (s["n"], misses, s["mean"], cm_to_in(s["mean"]),
                         s["min"], s["max"], bumper))
                if keep():
                    break
            writer.writerow({
                "actual_cm": cm, "actual_in": cm_to_in(cm),
                "readings": s["n"], "no_echo": misses,
                "mean_cm": s["mean"], "mean_in": cm_to_in(s["mean"]), "std_cm": s["std"],
                "min_cm": s["min"], "max_cm": s["max"],
                "error_pct": error_pct(s["mean"], cm), "esp_state": bumper,
            })
    finally:
        link.write(STOP_PACKET)
        link.close()


# ToF

def measure_tof(src):
    from navi_depth import zone_map
    dist, verdicts, valid = [], [], []
    got = 0
    while got < TOF_FRAMES:
        depth = src.read()
        if depth is None:
            continue
        got += 1
        center = zone_map(depth)["zones"]["center"]
        verdicts.append(center["verdict"])
        valid.append(center["valid_frac"] * 100.0)
        if center["distance_mm"] is not None:
            dist.append(center["distance_mm"] / 10.0)
    return dist, verdicts, valid


def run_tof(writer):
    from navi_depth import ArducamSource
    src = ArducamSource()
    print("%s\nUse a large flat target (box, door, wall) that fills the middle "
          "third of the view. Keep the floor out of the center zone: mount the "
          "camera at seat height or tilt it up slightly. Measure from the lens."
          % src.describe())
    steps = [(cm, "Place the target at %d cm (%.1f in)" % (cm, cm_to_in(cm)))
             for cm in TOF_CM]
    steps.append(("covered", "Cover the lens with your hand"))
    try:
        for actual, prompt in steps:
            if not ask(prompt):
                continue
            while True:
                dist, verdicts, valid = measure_tof(src)
                s = stats(dist)
                verdict = majority(verdicts)
                agree = 100.0 * verdicts.count(verdict) / len(verdicts)
                print("  center %s cm / %s in  (%s to %s cm)   valid %.0f%%   %s in %.0f%% of frames"
                      % (s["mean"], cm_to_in(s["mean"]), s["min"], s["max"],
                         np.mean(valid), verdict, agree))
                if keep():
                    break
            writer.writerow({
                "actual_cm": actual,
                "actual_in": cm_to_in(actual) if actual != "covered" else "",
                "frames": len(verdicts),
                "mean_cm": s["mean"], "mean_in": cm_to_in(s["mean"]), "std_cm": s["std"],
                "min_cm": s["min"], "max_cm": s["max"],
                "error_pct": error_pct(s["mean"], actual) if actual != "covered" else "",
                "valid_pct": round(float(np.mean(valid)), 1),
                "verdict": verdict, "verdict_agree_pct": round(agree, 1),
            })
    finally:
        src.close()


FIELDS = {
    "vision": ["step", "setup", "frames", "red_pct", "green_pct", "mean_ms",
               "std_ms", "p95_ms", "max_ms", "camera_fps", "image"],
    "uss": ["actual_cm", "actual_in", "readings", "no_echo", "mean_cm", "mean_in",
            "std_cm", "min_cm",
            "max_cm", "error_pct", "esp_state"],
    "tof": ["actual_cm", "actual_in", "frames", "mean_cm", "mean_in", "std_cm",
            "min_cm", "max_cm",
            "error_pct", "valid_pct", "verdict", "verdict_agree_pct"],
}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=sorted(FIELDS))
    ap.add_argument("--port", default=cfg.SERIAL_PORT)
    args = ap.parse_args()

    stamp = time.strftime("%Y%m%d-%H%M%S")
    out_dir = os.path.join(cfg.OUTPUT_DIR, "paper_%s_%s" % (args.mode, stamp))
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "results.csv")
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS[args.mode])
        writer.writeheader()

        class Flush:                       # keep finished rows if Ctrl C
            def writerow(self, row):
                writer.writerow(row)
                fh.flush()
        try:
            if args.mode == "vision":
                run_vision(Flush(), out_dir)
            elif args.mode == "uss":
                run_uss(Flush(), args.port)
            else:
                run_tof(Flush())
        except KeyboardInterrupt:
            print("\nstopped")
    print("\nsaved %s" % path)


if __name__ == "__main__":
    main()
