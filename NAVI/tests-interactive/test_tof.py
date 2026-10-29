"""Interactive ToF test - zone detection (blocked / caution / clear).

Run: python3 test_tof.py
"""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
import navi_config as cfg
from navi_depth import ArducamSource, zone_map, format_row, BLOCKED, CAUTION, CLEAR, UNKNOWN

def mm_to_in(mm):
    return mm / 25.4


import os, sys
import numpy as np

class LCD:
    """Minimal LCD framebuffer writer."""
    def __init__(self, fb="/dev/fb0"):
        self.fb = fb
        try:
            name = os.path.basename(fb)
            with open("/sys/class/graphics/%s/virtual_size" % name) as f:
                self.w, self.h = (int(v) for v in f.read().strip().split(","))
            with open("/sys/class/graphics/%s/bits_per_pixel" % name) as f:
                self.bpp = int(f.read().strip())
            self._fh = open(fb, "wb")
        except Exception:
            self._fh = None
            self.w, self.h, self.bpp = 320, 480, 16

    def text(self, lines, bg=(0,0,0)):
        """Show lines of text on the LCD. Each line is (text, color) or just text."""
        if not self._fh:
            return
        try:
            import cv2
            img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
            img[:] = bg
            y = 30
            for line in lines:
                if isinstance(line, tuple):
                    txt, color = line
                else:
                    txt, color = line, (255, 255, 255)
                cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                y += 25
                if y > self.h - 10:
                    break
            self._write_rgb(img)
        except Exception:
            pass

    def frame(self, cv_frame, overlay=None):
        """Show an OpenCV BGR frame on the LCD, with optional overlay text."""
        if not self._fh:
            return
        try:
            import cv2
            img = cv2.resize(cv_frame, (self.w, self.h))
            if overlay:
                for i, txt in enumerate(overlay):
                    cv2.putText(img, txt, (5, 20 + i*22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,255), 1)
            self._write_rgb(img)
        except Exception:
            pass

    def _write_rgb(self, bgr):
        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        r = (rgb[:,:,0].astype(np.uint16) >> 3) << 11
        g = (rgb[:,:,1].astype(np.uint16) >> 2) << 5
        b = rgb[:,:,2].astype(np.uint16) >> 3
        self._fh.seek(0)
        self._fh.write((r|g|b).astype("<u2").tobytes())
        self._fh.flush()

    def close(self):
        if self._fh:
            self._fh.close()
            self._fh = None

lcd = LCD()

try:
    src = ArducamSource()
except Exception as e:
    print(f"FAIL: cannot open ToF - {e}")
    sys.exit(1)

time.sleep(0.5)

print("=== ToF ZONE DETECTION TEST ===")
print(f"  Blocked: <{cfg.ZONE_BLOCKED_MM} mm ({mm_to_in(cfg.ZONE_BLOCKED_MM):.0f} in)")
print(f"  Caution: {cfg.ZONE_BLOCKED_MM}-{cfg.ZONE_CAUTION_MM} mm ({mm_to_in(cfg.ZONE_BLOCKED_MM):.0f}-{mm_to_in(cfg.ZONE_CAUTION_MM):.0f} in)")
print(f"  Clear:   >{cfg.ZONE_CAUTION_MM} mm ({mm_to_in(cfg.ZONE_CAUTION_MM):.0f} in)\n")

results = {}

def read_zones(src, n=10):
    """Read n frames through navi_depth.zone_map, the same rule the chair runs.

    Returns the center zone verdict seen most often, or None with no frames.
    """
    verdicts = []
    for _ in range(n):
        frame = src.read()
        if frame is None:
            continue
        summary = zone_map(frame)
        verdicts.append(summary["zones"]["center"]["verdict"])
        print("  " + format_row(summary))
        time.sleep(0.1)
    if not verdicts:
        return None
    return max(set(verdicts), key=verdicts.count)


def run_case(key, prompt, expected):
    input(prompt)
    verdict = read_zones(src)
    if verdict is None:
        results[key] = False
        print("  FAIL: no readings\n")
        return
    ok = verdict in expected
    results[key] = ok
    print("  Center: %s" % verdict)
    print("  %s: expected %s\n" % ("PASS" if ok else "FAIL", " or ".join(expected)))


run_case("close_blocked",
         "TEST 1: Hold a large object CLOSE, within 1m (39in), in front of the center. Press Enter...",
         (BLOCKED,))
run_case("mid_caution",
         "TEST 2: Move the object to 1-2m (39-79in) away. Press Enter...",
         (CAUTION,))
run_case("clear",
         "TEST 3: Clear the view - nothing within 2m (79in). Press Enter...",
         (CLEAR,))
# A hand on the lens may read BLOCKED or, with nothing valid, UNKNOWN. Both stop the chair.
run_case("hand_cover",
         "TEST 4: Cover the sensor with your hand right up against it. Press Enter...",
         (BLOCKED, UNKNOWN))

src.close()
passed = sum(results.values())
total = len(results)
print(f"=== ToF RESULT: {passed}/{total} passed ===")
summary = []
for name, ok in results.items():
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")
    summary.append(('PASS: %s' % name, (0,255,0)) if ok else ('FAIL: %s' % name, (0,0,255)))
lcd.text([('ToF RESULTS', (255,255,0)), ('', (0,0,0))] + summary)
lcd.close()
