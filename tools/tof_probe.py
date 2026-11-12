"""Show the raw numbers behind each ToF zone verdict.

    python3 tools/tof_probe.py            live
    python3 tools/tof_probe.py --save     also save the last depth map

The grid is the depth map at 12 x 6, nearest reading per cell in decimetres.
'..' no reading, '<<' closer than DEPTH_MIN_MM.
"""

import argparse
import os
import sys
import time

import numpy as np

# Tools sit one level below the modules they import.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import navi_config as cfg
from navi_depth import ArducamSource, zone_stats, ZONE_NAMES


def zone_detail(z):
    total = z.size
    nz = z[z > 0]
    valid = z[(z >= cfg.DEPTH_MIN_MM) & (z <= cfg.DEPTH_MAX_MM)]
    close = z[(z > 0) & (z < cfg.DEPTH_MIN_MM)]
    stats = zone_stats(z)
    return {
        "valid": 100.0 * valid.size / total,
        "close": 100.0 * close.size / total,
        "zero": 100.0 * (total - nz.size) / total,
        "min": int(nz.min()) if nz.size else None,
        "p10": int(np.percentile(valid, 10)) if valid.size else None,
        "median": int(np.median(valid)) if valid.size else None,
        "verdict": stats["verdict"],
    }


def grid(depth, cols=12, rows=6):
    h, w = depth.shape
    lines = []
    for r in range(rows):
        cells = []
        for c in range(cols):
            cell = depth[r * h // rows:(r + 1) * h // rows,
                         c * w // cols:(c + 1) * w // cols]
            nz = cell[cell > 0]
            if nz.size < cell.size * 0.05:
                cells.append("..")
            else:
                m = float(nz.min())
                cells.append("<<" if m < cfg.DEPTH_MIN_MM else "%02d" % min(99, int(m // 100)))
        # bars mark the left, center and right zone boundaries
        lines.append("   " + " ".join(cells[:4]) + " | " + " ".join(cells[4:8])
                     + " | " + " ".join(cells[8:]))
    return lines


def fmt(v):
    return "  -- " if v is None else "%5d" % v


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--save", action="store_true")
    args = ap.parse_args()

    src = ArducamSource()
    print(src.describe())
    last = None
    try:
        while True:
            depth = src.read()
            if depth is None:
                continue
            last = depth
            out = ["\033[H\033[J", "ToF probe   window %d..%d mm   blocked < %d   caution < %d   (Ctrl-C to stop)\n"
                   % (cfg.DEPTH_MIN_MM, cfg.DEPTH_MAX_MM, cfg.ZONE_BLOCKED_MM, cfg.ZONE_CAUTION_MM),
                   "  zone    valid%  close%  zero%    min    p10  median  verdict"]
            w = depth.shape[1]
            bounds = (0, w // 3, 2 * w // 3, w)
            for i, name in enumerate(ZONE_NAMES):
                d = zone_detail(depth[:, bounds[i]:bounds[i + 1]])
                out.append("  %-7s %6.0f  %6.0f  %5.0f  %s  %s   %s  %s"
                           % (name, d["valid"], d["close"], d["zero"],
                              fmt(d["min"]), fmt(d["p10"]), fmt(d["median"]),
                              d["verdict"]))
            out.append("\n  nearest reading per cell, decimetres (left | center | right):")
            out.extend(grid(depth))
            sys.stdout.write("\n".join(out) + "\n")
            sys.stdout.flush()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        if args.save and last is not None:
            os.makedirs(cfg.OUTPUT_DIR, exist_ok=True)
            path = os.path.join(cfg.OUTPUT_DIR, "depth_%s.npy" % time.strftime("%Y%m%d-%H%M%S"))
            np.save(path, last)
            print("\nsaved %s" % path)


if __name__ == "__main__":
    main()
