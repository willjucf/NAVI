"""Interactive webcam test with LCD display.

Shows live camera feed on the LCD with detection overlays.
Tests: green detected (go), red detected (stop), both (stop), clear (no detection).

Run: python3 test_webcam.py
"""
import sys, os, time

sys.path.insert(0, os.path.expanduser("~/mergetest"))

# Use the Pi's local X display (the LCD)
os.environ["DISPLAY"] = ":0"

import cv2
import numpy as np
import navi_config as cfg


# --- Detection ---

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
    g_best = max(g_contours, key=cv2.contourArea) if g_contours else None
    r_best = max(r_contours, key=cv2.contourArea) if r_contours else None
    g_area = cv2.contourArea(g_best) if g_best is not None else 0
    r_area = cv2.contourArea(r_best) if r_best is not None else 0

    green = g_area >= min_area
    red = r_area >= red_min
    return green, red, g_best if green else None, r_best if red else None


def annotate_frame(frame, green, red, g_contour, r_contour):
    out = frame.copy()
    if r_contour is not None:
        x, y, bw, bh = cv2.boundingRect(r_contour)
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 0, 255), 3)
        cv2.putText(out, "RED", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    if g_contour is not None:
        x, y, bw, bh = cv2.boundingRect(g_contour)
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
        cv2.putText(out, "GREEN", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    if red and green:
        status, color = "STOP (red overrides)", (0, 0, 255)
    elif red:
        status, color = "STOP (red detected)", (0, 0, 255)
    elif green:
        status, color = "GO (green detected)", (0, 255, 0)
    else:
        status, color = "CLEAR (no detection)", (200, 200, 200)

    cv2.rectangle(out, (0, 0), (out.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(out, status, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return out


def show_banner(win, text, color, w=480, h=320):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    y = h // 2 - 20
    for line in text:
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        y += 30
    cv2.imshow(win, img)
    cv2.waitKey(1)


def run_test(cap, win, label, expect_green, expect_red, duration=3.0):
    green_count = 0
    red_count = 0
    total = 0

    t = time.time()
    while time.time() - t < duration:
        ok, frame = cap.read()
        if not ok:
            continue
        green, red, gc, rc = detect(frame)
        shown = annotate_frame(frame, green, red, gc, rc)
        # Resize for LCD
        lcd_frame = cv2.resize(shown, (480, 320))
        cv2.imshow(win, lcd_frame)
        cv2.waitKey(1)
        green_count += green
        red_count += red
        total += 1

    green_rate = green_count / max(total, 1)
    red_rate = red_count / max(total, 1)

    if expect_green and not expect_red:
        ok = green_rate > 0.5 and red_rate < 0.3
    elif expect_red and not expect_green:
        ok = red_rate > 0.5
    elif expect_red and expect_green:
        ok = red_rate > 0.3 and green_rate > 0.3
    else:
        ok = green_rate < 0.2 and red_rate < 0.2

    return ok, green_rate, red_rate, total


# --- Main ---

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
for _ in range(10):
    cap.read()

WIN = "NAVI"
cv2.namedWindow(WIN, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

print("=== WEBCAM INTERACTIVE TEST (LCD) ===\n")
results = {}

# Test 1: Clear
show_banner(WIN, ["TEST 1: CLEAR", "No objects in view", "Press Enter in terminal"], (255, 255, 0))
input("TEST 1: Clear the view - no green or red objects. Press Enter...")
ok, gr, rr, n = run_test(cap, WIN, "CLEAR", False, False)
results["clear"] = ok
print(f"  Green: {gr*100:.0f}%  Red: {rr*100:.0f}%  ({n} frames)")
print(f"  {'PASS' if ok else 'FAIL'}: expected no detections\n")

# Test 2: Green
show_banner(WIN, ["TEST 2: GREEN", "Hold green object in view", "Press Enter in terminal"], (0, 255, 0))
input("TEST 2: Hold a GREEN object in view. Press Enter...")
ok, gr, rr, n = run_test(cap, WIN, "GREEN", True, False)
results["green_go"] = ok
print(f"  Green: {gr*100:.0f}%  Red: {rr*100:.0f}%  ({n} frames)")
print(f"  {'PASS' if ok else 'FAIL'}: expected GO (green detected)\n")

# Test 3: Red
show_banner(WIN, ["TEST 3: RED", "Hold red object in view", "Press Enter in terminal"], (0, 0, 255))
input("TEST 3: Hold a RED object in view. Press Enter...")
ok, gr, rr, n = run_test(cap, WIN, "RED", False, True)
results["red_stop"] = ok
print(f"  Green: {gr*100:.0f}%  Red: {rr*100:.0f}%  ({n} frames)")
print(f"  {'PASS' if ok else 'FAIL'}: expected STOP (red detected)\n")

# Test 4: Both
show_banner(WIN, ["TEST 4: BOTH", "Hold green AND red objects", "Press Enter in terminal"], (0, 165, 255))
input("TEST 4: Hold BOTH green and red objects in view. Press Enter...")
ok, gr, rr, n = run_test(cap, WIN, "BOTH", True, True)
results["both_stop"] = ok
print(f"  Green: {gr*100:.0f}%  Red: {rr*100:.0f}%  ({n} frames)")
print(f"  {'PASS' if ok else 'FAIL'}: expected STOP (red overrides green)\n")

cap.release()

# Show summary
passed = sum(results.values())
total = len(results)
summary = ["RESULT: %d/%d passed" % (passed, total)] + [
    "%s: %s" % ("PASS" if ok else "FAIL", name) for name, ok in results.items()
]
show_banner(WIN, summary, (0, 255, 0) if passed == total else (0, 0, 255))
time.sleep(3)

print(f"=== WEBCAM RESULT: {passed}/{total} passed ===")
for name, ok in results.items():
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")

cv2.destroyAllWindows()
