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

import glob
PORT = (glob.glob("/dev/serial/by-id/*CP210*") or ["/dev/ttyUSB0"])[0]
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

passed = sum(results.values())
total = len(results)
print(f"=== USS RESULT: {passed}/{total} passed ===")
for name, ok in results.items():
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")
