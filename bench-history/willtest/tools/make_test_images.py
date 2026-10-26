"""Generate the synthetic test frames into test_images/.

Report 6.11.1 uses three synthetic solid colour frames to demonstrate the
decision logic, and explains why they are synthetic rather than photographs:
removing lighting and camera noise leaves only the logic, so what the figures
show is the decision itself.

This regenerates those three, plus a glare case, so that the image based paths
(navi_vision.py --source images, navi_control.py --source images) work on a
fresh clone with no camera and no photographs to hand.

Real photographs belong in the same folder and are the more valuable input. This
only exists so the folder is never empty.

    python3 tools/make_test_images.py
"""

import os
import sys

import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

import navi_config as cfg
from fixtures import (green_frame, red_frame, red_and_green_frame, glare_frame,
                      blank_frame)


FRAMES = {
    # The three cases report 6.11.1 shows figures for.
    "synthetic_green": green_frame(cx_frac=0.5),
    "synthetic_red": red_frame(cx_frac=0.5),
    "synthetic_both": red_and_green_frame(),
    # Steering cases, so a cycled run exercises more than straight ahead.
    "synthetic_green_left": green_frame(cx_frac=0.2),
    "synthetic_green_right": green_frame(cx_frac=0.8),
    # The glare regression from 6.11.1 and the FMEA in 6.21. Included so a
    # cycled run visibly declines to move on it.
    "synthetic_glare": glare_frame(),
    "synthetic_empty": blank_frame(),
}


def main():
    os.makedirs(cfg.INPUT_DIR, exist_ok=True)
    for name, frame in FRAMES.items():
        path = os.path.join(cfg.INPUT_DIR, name + ".png")
        cv2.imwrite(path, frame)
        print("wrote %s" % path)
    print("\n%d synthetic frames in %s" % (len(FRAMES), cfg.INPUT_DIR))
    print("Add real photographs to the same folder. hsv_probe.py skips anything "
          "named synthetic_*, since perfect colours are not worth sampling.")


if __name__ == "__main__":
    main()
