"""Interactive ToF test - zone detection (blocked / caution / clear).

Run: python3 test_tof.py
"""
import sys, os, time
sys.path.insert(0, os.path.expanduser("~/usb-bench"))
import numpy as np
import navi_config as cfg
from navi_depth import ArducamSource

def mm_to_in(mm):
    return mm / 25.4

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
for name, ok in results.items():
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")
