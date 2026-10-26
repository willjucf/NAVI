"""Guided ToF capture: press Enter at each hand position, the data is saved.

Walks through a fixed list of positions (finger and flat hand at several
distances, left, center and right). At each one you put your hand in place and
press Enter, and about a second of depth frames is recorded. Everything goes to
output/tof_capture_<time>/ so it can be analysed afterwards:

    summary.json   per position: zone stats, percentiles, confidence, a coarse
                   distance grid, and what navi_depth decided
    frames.npz     the raw depth and confidence frames for every position

    python3 tools/tof_capture.py

At the prompt:
    Enter   record this position
    s       skip it
    r       redo the previous one
    q       stop early (everything recorded so far is still saved)
"""

import datetime
import json
import os
import sys
import threading
import time

import numpy as np

# Tools live in tools/, one level below the modules they use.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import navi_config as cfg
from navi_depth import ArducamSource, zone_map, ZONE_NAMES

STEPS = [
    ("baseline",        "Nothing in front of the ToF, just point it at the room"),
    ("finger_c_05",     "Fingertip, CENTER, about 5 cm from the lens"),
    ("finger_c_15",     "Fingertip, CENTER, about 15 cm"),
    ("finger_c_30",     "Fingertip, CENTER, about 30 cm"),
    ("finger_c_60",     "Fingertip, CENTER, about 60 cm"),
    ("hand_c_05",       "Flat open hand, CENTER, about 5 cm"),
    ("hand_c_15",       "Flat open hand, CENTER, about 15 cm"),
    ("hand_c_30",       "Flat open hand, CENTER, about 30 cm"),
    ("hand_c_60",       "Flat open hand, CENTER, about 60 cm"),
    ("hand_c_100",      "Flat open hand, CENTER, about 1 m"),
    ("hand_l_30",       "Flat open hand, LEFT side of view, about 30 cm"),
    ("hand_r_30",       "Flat open hand, RIGHT side of view, about 30 cm"),
    ("baseline_end",    "Nothing in front again (checks nothing drifted)"),
]

FRAMES_PER_STEP = 20      # about 0.7 s at 30 fps
FRAMES_SAVED = 10         # raw frames kept per step, to keep the file small


class Grabber(threading.Thread):
    """Keeps the camera streaming so a capture starts the instant Enter is hit."""

    def __init__(self, src):
        super().__init__(daemon=True)
        self.src = src
        self.lock = threading.Lock()
        self.frames = []
        self.recording = 0
        self.running = True
        self.error = None

    def run(self):
        cam = self.src.cam
        try:
            while self.running:
                f = cam.requestFrame(200)
                if f is None:
                    continue
                try:
                    depth = np.asarray(f.depth_data, dtype=np.float32).copy()
                    conf = np.asarray(f.confidence_data, dtype=np.float32).copy()
                finally:
                    cam.releaseFrame(f)
                with self.lock:
                    if self.recording > 0:
                        self.frames.append((depth, conf))
                        self.recording -= 1
        except Exception as exc:
            self.error = exc

    def record(self, n):
        with self.lock:
            self.frames = []
            self.recording = n
        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self.lock:
                if self.recording == 0:
                    return list(self.frames)
            time.sleep(0.02)
        with self.lock:
            self.recording = 0
            return list(self.frames)


def pct(a, q):
    return None if a.size == 0 else int(np.percentile(a, q))


def zone_block(depth_stack, conf_stack, x0, x1):
    z = depth_stack[:, :, x0:x1]
    c = conf_stack[:, :, x0:x1]
    total = z.size
    nz = z[z > 0]
    valid = z[(z >= cfg.DEPTH_MIN_MM) & (z <= cfg.DEPTH_MAX_MM)]
    close = z[(z > 0) & (z < cfg.DEPTH_MIN_MM)]
    per_frame_min = [int(f[f > 0].min()) if (f > 0).any() else None for f in z]
    return {
        "valid_pct": round(100.0 * valid.size / total, 1),
        "close_pct": round(100.0 * close.size / total, 1),
        "zero_pct": round(100.0 * (total - nz.size) / total, 1),
        "min": int(nz.min()) if nz.size else None,
        "p1": pct(valid, 1), "p5": pct(valid, 5), "p10": pct(valid, 10),
        "p25": pct(valid, 25), "median": pct(valid, 50),
        "close_min": int(close.min()) if close.size else None,
        "conf_mean": round(float(c.mean()), 1),
        "conf_mean_valid": round(float(c[(z >= cfg.DEPTH_MIN_MM) & (z <= cfg.DEPTH_MAX_MM)].mean()), 1) if valid.size else None,
        "per_frame_min": per_frame_min,
    }


def coarse_grid(depth_med, cols=12, rows=6):
    h, w = depth_med.shape
    g = []
    for r in range(rows):
        row = []
        for c in range(cols):
            cell = depth_med[r * h // rows:(r + 1) * h // rows,
                             c * w // cols:(c + 1) * w // cols]
            nz = cell[cell > 0]
            row.append(int(nz.min()) if nz.size >= cell.size * 0.05 else 0)
        g.append(row)
    return g


def summarise(label, prompt, frames):
    depth = np.stack([f[0] for f in frames])
    conf = np.stack([f[1] for f in frames])
    w = depth.shape[2]
    bounds = (0, w // 3, 2 * w // 3, w)
    depth_med = np.median(depth, axis=0)
    decided = zone_map(depth_med)
    out = {"label": label, "prompt": prompt, "frames": len(frames),
           "shape": list(depth.shape[1:]), "zones": {},
           "navi_decision": {
               "path_blocked": bool(decided["path_blocked"]),
               "reason": decided.get("reason"),
               "zones": {n: {"distance_mm": decided["zones"][n]["distance_mm"],
                             "verdict": decided["zones"][n]["verdict"]}
                         for n in ZONE_NAMES}},
           "grid_mm": coarse_grid(depth_med)}
    for i, name in enumerate(ZONE_NAMES):
        out["zones"][name] = zone_block(depth, conf, bounds[i], bounds[i + 1])
    return out, depth, conf


def one_line(s):
    c = s["zones"]["center"]
    d = s["navi_decision"]["zones"]
    return ("center: min %s  p10 %s  median %s  close %s%%  zero %s%%   ->  NAVI says L:%s C:%s R:%s"
            % (c["min"], c["p10"], c["median"], c["close_pct"], c["zero_pct"],
               d["left"]["verdict"], d["center"]["verdict"], d["right"]["verdict"]))


def save(outdir, results, raw):
    with open(os.path.join(outdir, "summary.json"), "w") as fh:
        json.dump({"config": {"DEPTH_MIN_MM": cfg.DEPTH_MIN_MM,
                              "DEPTH_MAX_MM": cfg.DEPTH_MAX_MM,
                              "ZONE_BLOCKED_MM": cfg.ZONE_BLOCKED_MM,
                              "ZONE_CAUTION_MM": cfg.ZONE_CAUTION_MM,
                              "ZONE_MIN_VALID_FRAC": cfg.ZONE_MIN_VALID_FRAC},
                   "steps": results}, fh, indent=1)
    arrays = {}
    for label, (depth, conf) in raw.items():
        arrays[label + "_depth"] = np.clip(depth[:FRAMES_SAVED], 0, 65535).astype(np.uint16)
        arrays[label + "_conf"] = conf[:FRAMES_SAVED].astype(np.float16)
    np.savez_compressed(os.path.join(outdir, "frames.npz"), **arrays)


def main():
    outdir = os.path.join(cfg.OUTPUT_DIR, "tof_capture_%s"
                          % datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(outdir, exist_ok=True)

    src = ArducamSource()
    grab = Grabber(src)
    grab.start()
    time.sleep(1.0)       # let exposure settle
    if grab.error:
        sys.exit("camera error: %s" % grab.error)

    print("ToF capture. %d positions. Enter = record, s = skip, r = redo previous, q = quit.\n"
          % len(STEPS))
    results, raw = [], {}
    i = 0
    try:
        while i < len(STEPS):
            label, prompt = STEPS[i]
            ans = input("[%d/%d] %s\n      ready? " % (i + 1, len(STEPS), prompt)).strip().lower()
            if ans == "q":
                break
            if ans == "s":
                print("      skipped\n")
                i += 1
                continue
            if ans == "r":
                if i > 0:
                    i -= 1
                    results = [r for r in results if r["label"] != STEPS[i][0]]
                    raw.pop(STEPS[i][0], None)
                print()
                continue
            frames = grab.record(FRAMES_PER_STEP)
            if not frames:
                print("      no frames arrived, try again\n")
                continue
            s, depth, conf = summarise(label, prompt, frames)
            results.append(s)
            raw[label] = (depth, conf)
            save(outdir, results, raw)          # save as we go
            print("      recorded %d frames. %s\n" % (len(frames), one_line(s)))
            i += 1
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        grab.running = False
        grab.join(timeout=1.0)
        src.close()

    if results:
        save(outdir, results, raw)
        print("Saved %d positions to %s" % (len(results), outdir))
    else:
        print("Nothing recorded.")


if __name__ == "__main__":
    main()
