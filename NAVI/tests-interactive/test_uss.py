"""Interactive ultrasonic test - multiple standardized distances.

Run: python3 test_uss.py
"""
import serial, time, re, sys

def mm_to_in(mm):
    return mm / 25.4

DISTANCES = [
    ("30cm (12in)",  300,  200,  450),
    ("50cm (20in)",  500,  350,  650),
    ("1m (39in)",   1000,  750, 1300),
    ("2m (79in)",   2000, 1500, 2600),
]

PORT = "/dev/ttyUSB0"
BAUD = 115200

def read_uss(seconds=4):
    s = serial.Serial()
    s.port = PORT; s.baudrate = BAUD; s.timeout = 0.2
    s.dtr = False; s.rts = False; s.open()
    s.reset_input_buffer()
    t = time.time(); buf = b""
    while time.time() - t < seconds:
        buf += s.read(8192)
    s.close()
    readings = [int(r) for r in re.findall(r"uss=(-?\d+)mm", buf.decode(errors="replace"))]
    return readings


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

print("=== ULTRASONIC INTERACTIVE TEST ===")
print(f"  Sensor: RCWL-1655 on {PORT}")
print(f"  Valid range: 20-4000 mm (0.8-157 in)")
print(f"  -1 = no reading (nothing in range or no echo)\n")

results = {}

# No object test first
input("TEST: Clear the area - nothing within 1m (39in) in front of the sensor. Press Enter...")
readings = read_uss(4)
valid = [r for r in readings if r > 0]
ok = len(valid) <= 2
results["clear"] = ok
print(f"  Total: {len(readings)}, valid: {len(valid)}")
if valid:
    print(f"  Readings: {min(valid)}-{max(valid)} mm ({mm_to_in(min(valid)):.1f}-{mm_to_in(max(valid)):.1f} in)")
print(f"  {'PASS' if ok else 'FAIL'}: expected mostly no-reading when clear\n")

# Distance tests
for name, target, lo, hi in DISTANCES:
    input(f"TEST: Place a flat object at {name} from the sensor, aimed straight. Press Enter...")
    readings = read_uss(6)
    valid = [r for r in readings if r > 0]

    if not valid:
        print(f"  FAIL: no valid readings (all -1)")
        print(f"  Check aim - beam is narrow (~20 deg)\n")
        results[name] = False
        continue

    median = int(sorted(valid)[len(valid)//2])
    mn, mx = min(valid), max(valid)
    hit_rate = len(valid) / len(readings) * 100
    ok = lo <= median <= hi
    results[name] = ok
    print(f"  Valid: {len(valid)}/{len(readings)} ({hit_rate:.0f}%)")
    print(f"  Range: {mn}-{mx} mm ({mm_to_in(mn):.1f}-{mm_to_in(mx):.1f} in), median: {median} mm ({mm_to_in(median):.1f} in)")
    print(f"  Expected: {lo}-{hi} mm ({mm_to_in(lo):.1f}-{mm_to_in(hi):.1f} in)")
    print(f"  {'PASS' if ok else 'FAIL'}: {name}\n")
    lcd.text([('USS: %s' % name, (255,255,0)),
              ('median: %dmm' % median, (255,255,255)),
              ('PASS' if ok else 'FAIL', (0,255,0) if ok else (0,0,255))])

passed = sum(results.values())
total = len(results)
print(f"=== USS RESULT: {passed}/{total} passed ===")
summary = []
for name, ok in results.items():
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")
    summary.append(('PASS: %s' % name, (0,255,0)) if ok else ('FAIL: %s' % name, (0,0,255)))
lcd.text([('USS RESULTS', (255,255,0)), ('', (0,0,0))] + summary)
lcd.close()
