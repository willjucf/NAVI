"""Guided color capture. Press Enter at each prompt to save photos and color data.

For debugging "it misses green things and sees red that is not there". At each
step hold the object in the CENTER of the view and press Enter. Each step saves:

    <step>_raw.jpg        what the camera saw
    <step>_detect.jpg     same frame with the detector's decision drawn on
    <step>_masks.jpg      green mask (left) and red mask (right), white is a match
    summary.json          HSV stats of the center box, how much passed the
                          green and red thresholds, and what detect() decided
                          on 5 frames in a row

Everything goes to output/color_capture_<time>/.

    python3 tools/color_capture.py

At the prompt:
    Enter           record
    text then Enter record and keep the text as a note ("teal cup")
    s               skip
    r               redo the previous step
    q               stop early (everything so far is saved)
"""

import datetime
import json
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

# Tools sit one level below the modules they import.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import navi_config as cfg
from navi_vision import CameraSource, normalize, detect, annotate

STEPS = [
    ("empty",        "Nothing special in view. Point the camera at the room as it normally would be"),
    ("green1_50",    "Your main GREEN target, CENTER of view, about 50 cm away"),
    ("green1_100",   "Same GREEN target, CENTER, about 1 m"),
    ("green1_200",   "Same GREEN target, CENTER, about 2 m"),
    ("green2_100",   "A DIFFERENT green object if you have one, CENTER, about 1 m (s to skip)"),
    ("red1_50",      "Your RED target, CENTER, about 50 cm"),
    ("red1_100",     "Same RED target, CENTER, about 1 m"),
    ("hand_50",      "Just your bare HAND (palm) in the CENTER, about 50 cm, nothing else"),
    ("falsered_1",   "Point the camera at something that gave a FALSE red (type what it is, then Enter)"),
    ("falsered_2",   "Another false red trigger if you noticed one (type what it is, or s)"),
    ("missgreen_1",  "A green thing it MISSED, CENTER (type what it is, or s)"),
    ("empty_end",    "Nothing special in view again"),
]

CENTER_FRAC = 0.30        # center box is the middle 30% of width and height
FRAMES_PER_STEP = 5


class Grabber(threading.Thread):
    def __init__(self, src):
        super().__init__(daemon=True)
        self.src = src
        self.lock = threading.Lock()
        self.latest = None
        self.seq = 0
        self.running = True

    def run(self):
        while self.running:
            frame, _ = self.src.read()
            if frame is None:
                continue
            with self.lock:
                self.latest = frame
                self.seq += 1

    def next_frames(self, n):
        out, last = [], -1
        deadline = time.time() + 5
        while len(out) < n and time.time() < deadline:
            with self.lock:
                if self.seq != last and self.latest is not None:
                    out.append(self.latest.copy())
                    last = self.seq
            time.sleep(0.03)
        return out


def pcts(a):
    if a.size == 0:
        return None
    return [int(x) for x in np.percentile(a, [5, 25, 50, 75, 95])]


def center_box(img):
    h, w = img.shape[:2]
    bw, bh = int(w * CENTER_FRAC), int(h * CENTER_FRAC)
    x0, y0 = (w - bw) // 2, (h - bh) // 2
    return x0, y0, bw, bh


def colour_stats(frame):
    """HSV statistics of the center box, split by rough hue family."""
    x0, y0, bw, bh = center_box(frame)
    hsv = cv2.cvtColor(frame[y0:y0 + bh, x0:x0 + bw], cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].ravel(), hsv[..., 1].ravel(), hsv[..., 2].ravel()
    total = H.size

    def band(mask):
        return {"pct_of_box": round(100.0 * mask.sum() / total, 1),
                "H": pcts(H[mask]), "S": pcts(S[mask]), "V": pcts(V[mask])}

    colourful = S >= 40                     # ignore grays and whites
    return {
        "box_all": {"H": pcts(H), "S": pcts(S), "V": pcts(V)},
        "greenish_H25_95": band(colourful & (H >= 25) & (H <= 95)),
        "reddish_H0_12_or_165_179": band(colourful & ((H <= 12) | (H >= 165))),
        "orange_yellow_H12_25": band(colourful & (H > 12) & (H < 25)),
        "blue_cyan_H95_130": band(colourful & (H > 95) & (H <= 130)),
    }


def mask_coverage(mask):
    x0, y0, bw, bh = center_box(mask)
    box = mask[y0:y0 + bh, x0:x0 + bw]
    return round(100.0 * (box > 0).sum() / box.size, 1)


def camera_controls():
    try:
        out = subprocess.run(["v4l2-ctl", "-d", "/dev/video%d" % cfg.CAMERA_INDEX, "-l"],
                             capture_output=True, text=True, timeout=3).stdout
        keep = ("exposure", "white_balance", "gain", "brightness", "saturation",
                "contrast", "backlight", "power_line")
        return [l.strip() for l in out.splitlines() if any(k in l for k in keep)]
    except Exception:
        return []


def record_step(outdir, label, prompt, note, frames):
    decisions = []
    for f in frames:
        d = detect(normalize(f))
        decisions.append({"status": d["status"], "packet": d["packet"],
                          "green_area": d["green_area"], "red_area": d["red_area"]})
    mid = normalize(frames[len(frames) // 2])
    d = detect(mid)

    x0, y0, bw, bh = center_box(mid)
    raw = mid.copy()
    cv2.rectangle(raw, (x0, y0), (x0 + bw, y0 + bh), (255, 255, 255), 1)
    cv2.imwrite(os.path.join(outdir, label + "_raw.jpg"), raw)
    cv2.imwrite(os.path.join(outdir, label + "_detect.jpg"), annotate(mid, d))
    g = cv2.cvtColor(d["green_mask"], cv2.COLOR_GRAY2BGR)
    r = cv2.cvtColor(d["red_mask"], cv2.COLOR_GRAY2BGR)
    g[:, :, (0, 2)] = 0                     # tint green mask green
    r[:, :, (0, 1)] = 0                     # tint red mask red
    both = np.hstack([g, r])
    cv2.imwrite(os.path.join(outdir, label + "_masks.jpg"),
                cv2.resize(both, (both.shape[1] // 2, both.shape[0] // 2)))

    return {
        "label": label, "prompt": prompt, "note": note,
        "frame_size": [mid.shape[1], mid.shape[0]],
        "min_area_px": int(cfg.MIN_AREA_FRAC * mid.shape[0] * mid.shape[1]),
        "decisions": decisions,
        "center_box_green_mask_pct": mask_coverage(d["green_mask"]),
        "center_box_red_mask_pct": mask_coverage(d["red_mask"]),
        "colour": colour_stats(mid),
    }


def main():
    outdir = os.path.join(cfg.OUTPUT_DIR, "color_capture_%s"
                          % datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(outdir, exist_ok=True)

    src = CameraSource()
    grab = Grabber(src)
    grab.start()
    time.sleep(1.5)       # let auto exposure and white balance settle

    meta = {"thresholds": {k: getattr(cfg, k).tolist() for k in
                           ("LOWER_GREEN", "UPPER_GREEN", "LOWER_RED_1", "UPPER_RED_1",
                            "LOWER_RED_2", "UPPER_RED_2")},
            "MIN_AREA_FRAC": cfg.MIN_AREA_FRAC, "camera_controls": camera_controls()}

    print("Colour capture, %d steps. Hold things in the CENTER of the view.\n"
          "Enter = record (type a note first if you like), s = skip, r = redo, q = quit.\n"
          % len(STEPS))
    results = []
    i = 0
    try:
        while i < len(STEPS):
            label, prompt = STEPS[i]
            ans = input("[%d/%d] %s\n      ready? " % (i + 1, len(STEPS), prompt)).strip()
            low = ans.lower()
            if low == "q":
                break
            if low == "s":
                print("      skipped\n")
                i += 1
                continue
            if low == "r":
                if i > 0:
                    i -= 1
                    results = [x for x in results if x["label"] != STEPS[i][0]]
                print()
                continue
            frames = grab.next_frames(FRAMES_PER_STEP)
            if not frames:
                print("      no frames from the camera, try again\n")
                continue
            res = record_step(outdir, label, prompt, ans, frames)
            results.append(res)
            with open(os.path.join(outdir, "summary.json"), "w") as fh:
                json.dump({"meta": meta, "steps": results}, fh, indent=1)
            dec = res["decisions"][len(frames) // 2]
            print("      saved. detector said: %s   (green in box %s%%, red in box %s%%)\n"
                  % (dec["status"], res["center_box_green_mask_pct"],
                     res["center_box_red_mask_pct"]))
            i += 1
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        grab.running = False
        grab.join(timeout=1.0)
        src.release()

    if results:
        print("Saved %d steps to %s" % (len(results), outdir))
    else:
        print("Nothing recorded.")


if __name__ == "__main__":
    main()
