"""Interactive webcam test - green detection, red detection, no object.

Run: python3 test_webcam.py
"""
import sys, os, time
sys.path.insert(0, os.path.expanduser("~/usb-bench"))
import cv2
import numpy as np
import navi_config as cfg

def clean_mask(mask):
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    return cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)

def detect(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green_mask = clean_mask(cv2.inRange(hsv, cfg.LOWER_GREEN, cfg.UPPER_GREEN))
    red_mask = clean_mask(cv2.bitwise_or(
        cv2.inRange(hsv, cfg.LOWER_RED_1, cfg.UPPER_RED_1),
        cv2.inRange(hsv, cfg.LOWER_RED_2, cfg.UPPER_RED_2)))
    h, w = frame.shape[:2]
    min_area = cfg.MIN_AREA_FRAC * w * h
    red_min = getattr(cfg, "RED_MIN_AREA_FRAC", cfg.MIN_AREA_FRAC) * w * h
    g_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    r_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    g_area = max((cv2.contourArea(c) for c in g_contours), default=0)
    r_area = max((cv2.contourArea(c) for c in r_contours), default=0)
    green_pct = 100 * g_area / (w * h)
    red_pct = 100 * r_area / (w * h)
    green_det = g_area >= min_area
    red_det = r_area >= red_min
    return green_det, red_det, green_pct, red_pct

def sample(cap, n=10):
    greens, reds, gp, rp = 0, 0, [], []
    for _ in range(n):
        ok, frame = cap.read()
        if not ok: continue
        gd, rd, gpct, rpct = detect(frame)
        greens += gd; reds += rd; gp.append(gpct); rp.append(rpct)
        time.sleep(0.05)
    return greens, reds, n, gp, rp

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
for _ in range(10): cap.read()

print("=== WEBCAM INTERACTIVE TEST ===\n")
results = {}

# Test 1: No object
input("TEST 1: Clear the view - no green or red objects. Press Enter...")
g, r, n, gp, rp = sample(cap)
ok = g == 0 and r == 0
results["no_object"] = ok
print(f"  Green detected: {g}/{n}  Red detected: {r}/{n}")
print(f"  Green area: {max(gp):.2f}%  Red area: {max(rp):.2f}%")
print(f"  {'PASS' if ok else 'FAIL'}: expected no detections\n")

# Test 2: Green
input("TEST 2: Hold a GREEN object in view. Press Enter...")
g, r, n, gp, rp = sample(cap)
ok = g >= 7
results["green"] = ok
print(f"  Green detected: {g}/{n}  Red detected: {r}/{n}")
print(f"  Green area: {max(gp):.2f}%")
print(f"  {'PASS' if ok else 'FAIL'}: expected green in >=7/{n} frames\n")

# Test 3: Red
input("TEST 3: Hold a RED object in view. Press Enter...")
g, r, n, gp, rp = sample(cap)
ok = r >= 7
results["red"] = ok
print(f"  Green detected: {g}/{n}  Red detected: {r}/{n}")
print(f"  Red area: {max(rp):.2f}%")
print(f"  {'PASS' if ok else 'FAIL'}: expected red in >=7/{n} frames\n")

cap.release()
passed = sum(results.values())
print(f"=== WEBCAM RESULT: {passed}/3 passed ===")
for name, ok in results.items():
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")
