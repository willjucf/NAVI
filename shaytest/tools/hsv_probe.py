import cv2
import numpy as np
import os
import glob

# HSV calibration probe, read only.
# For each real test image, reports the saturation and value spread of reddish
# and greenish pixels, so thresholds come from measured data instead of
# guesses. Synthetic images are skipped since their colors are not real.

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INPUT_DIR = os.path.join(SCRIPT_DIR, "test_images")

# Old ranges, for comparison
CUR_GREEN = (np.array([35, 30, 50]), np.array([90, 255, 255]))
CUR_RED_1 = (np.array([0, 210, 100]), np.array([10, 255, 255]))
CUR_RED_2 = (np.array([170, 210, 100]), np.array([179, 255, 255]))


def pct(arr):
    # min, 5th, 25th, 50th, 75th, 95th and max percentiles
    if arr.size == 0:
        return None
    p = np.percentile(arr, [0, 5, 25, 50, 75, 95, 100])
    return [int(round(x)) for x in p]


def fmt(label, s_vals, v_vals):
    n = s_vals.size
    if n == 0:
        print(f"    {label:12s} no pixels in this hue band")
        return
    sp = pct(s_vals)
    vp = pct(v_vals)
    print(f"    {label:12s} count={n}")
    print(f"      Saturation  min={sp[0]:3d}  5%={sp[1]:3d}  25%={sp[2]:3d}  "
          f"med={sp[3]:3d}  75%={sp[4]:3d}  95%={sp[5]:3d}  max={sp[6]:3d}")
    print(f"      Value       min={vp[0]:3d}  5%={vp[1]:3d}  25%={vp[2]:3d}  "
          f"med={vp[3]:3d}  75%={vp[4]:3d}  95%={vp[5]:3d}  max={vp[6]:3d}")


def main():
    patterns = ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG", "*.PNG")
    paths = []
    for p in patterns:
        paths.extend(glob.glob(os.path.join(INPUT_DIR, p)))
    # Skip synthetic images, their perfect colors are not worth sampling
    paths = sorted(set(p for p in paths
                       if not os.path.basename(p).lower().startswith("synthetic")))

    if not paths:
        print("No real images found in test_images/ (only synthetic ones?).")
        return

    print(f"Probing {len(paths)} real image(s) from {INPUT_DIR}\n")

    for path in paths:
        name = os.path.basename(path)
        frame = cv2.imread(path)
        if frame is None:
            print(f"{name}: [SKIP] could not read")
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

        # Hue only bands so we see the whole spread, but drop near gray pixels
        # (S under 20) so background does not skew the stats.
        colored = S >= 20
        red_hue = ((H <= 15) | (H >= 165)) & colored
        green_hue = ((H >= 35) & (H <= 90)) & colored

        print(f"{name}")
        fmt("RED hue", S[red_hue], V[red_hue])
        fmt("GREEN hue", S[green_hue], V[green_hue])

        # How many pixels the old ranges catch, as a sanity check
        g_mask = cv2.inRange(hsv, *CUR_GREEN)
        r_mask = cv2.bitwise_or(cv2.inRange(hsv, *CUR_RED_1),
                                cv2.inRange(hsv, *CUR_RED_2))
        print(f"    current-range catch:  GREEN={cv2.countNonZero(g_mask)}  "
              f"RED={cv2.countNonZero(r_mask)}")
        print()


if __name__ == "__main__":
    main()
