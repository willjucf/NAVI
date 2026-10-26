"""Interactive ToF test - zone detection (blocked / caution / clear).

Run: python3 test_tof.py
"""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
import navi_config as cfg
from navi_depth import ArducamSource

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
    """Read n frames, return avg % of pixels in each zone."""
    blocked_pcts = []
    caution_pcts = []
    clear_pcts = []
    for _ in range(n):
        frame = src.read()
        if frame is None:
            continue
        valid = frame[(frame >= cfg.DEPTH_MIN_MM) & (frame <= cfg.DEPTH_MAX_MM)]
        if valid.size == 0:
            continue
        total = valid.size
        blocked_pcts.append(100 * (valid < cfg.ZONE_BLOCKED_MM).sum() / total)
        caution_pcts.append(100 * ((valid >= cfg.ZONE_BLOCKED_MM) & (valid < cfg.ZONE_CAUTION_MM)).sum() / total)
        clear_pcts.append(100 * (valid >= cfg.ZONE_CAUTION_MM).sum() / total)
        time.sleep(0.1)
    if not blocked_pcts:
        return None, None, None
    return np.mean(blocked_pcts), np.mean(caution_pcts), np.mean(clear_pcts)

# Test 1: Object close - should be BLOCKED
input("TEST 1: Hold a large object CLOSE, within 1m (39in). Press Enter...")
b, c, cl = read_zones(src)
if b is not None:
    ok = b > 15
    results["close_blocked"] = ok
    print(f"  Blocked: {b:.0f}%  Caution: {c:.0f}%  Clear: {cl:.0f}%")
    print(f"  {'PASS' if ok else 'FAIL'}: expected >15% blocked pixels\n")
else:
    results["close_blocked"] = False
    print("  FAIL: no readings\n")

# Test 2: Object mid-range - should be CAUTION
input("TEST 2: Move the object to 1-2m (39-79in) away. Press Enter...")
b, c, cl = read_zones(src)
if b is not None:
    ok = c > 10
    results["mid_caution"] = ok
    print(f"  Blocked: {b:.0f}%  Caution: {c:.0f}%  Clear: {cl:.0f}%")
    print(f"  {'PASS' if ok else 'FAIL'}: expected >10% caution pixels\n")
else:
    results["mid_caution"] = False
    print("  FAIL: no readings\n")

# Test 3: Nothing close - should be CLEAR
input("TEST 3: Clear the view - nothing within 2m (79in). Press Enter...")
b, c, cl = read_zones(src)
if b is not None:
    ok = cl > 50 and b < 20
    results["clear"] = ok
    print(f"  Blocked: {b:.0f}%  Caution: {c:.0f}%  Clear: {cl:.0f}%")
    print(f"  {'PASS' if ok else 'FAIL'}: expected >50% clear, <10% blocked\n")
else:
    results["clear"] = False
    print("  FAIL: no readings\n")

# Test 4: Verify change - cover the sensor
input("TEST 4: Cover the sensor with your hand right up against it. Press Enter...")
b, c, cl = read_zones(src)
if b is not None:
    ok = b > 30
    results["hand_cover"] = ok
    print(f"  Blocked: {b:.0f}%  Caution: {c:.0f}%  Clear: {cl:.0f}%")
    print(f"  {'PASS' if ok else 'FAIL'}: expected >30% blocked\n")
else:
    results["hand_cover"] = False
    print("  FAIL: no readings\n")

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
