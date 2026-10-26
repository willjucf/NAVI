"""Interactive ESP link test - packet send/receive, watchdog, e-stop.

Run: python3 test_esp_link.py
"""
import serial, time, re, sys

PORT = "/dev/ttyUSB0"
BAUD = 115200

def open_port():
    s = serial.Serial()
    s.port = PORT; s.baudrate = BAUD; s.timeout = 0.2
    s.dtr = False; s.rts = False; s.open()
    return s

def send_sustained(s, packet, duration=2.0):
    """Send a packet at 20 Hz for duration seconds, return log text."""
    buf = b""
    t = time.time()
    while time.time() - t < duration:
        s.write(packet)
        time.sleep(0.05)
        buf += s.read(4096)
    buf += s.read(4096)
    return buf.decode(errors="replace")

def read_lines(s, seconds=2):
    buf = b""
    t = time.time()
    while time.time() - t < seconds:
        buf += s.read(4096)
    return buf.decode(errors="replace")

def get_state(txt):
    m = re.findall(r"state=(\S+)", txt)
    return m[-1] if m else None

def get_pwm(txt):
    m = re.findall(r"pwm=(-?\d+),(-?\d+)", txt)
    return (int(m[-1][0]), int(m[-1][1])) if m else None

def establish_link(s):
    """Send packets until the link is up and not halted. Returns final state."""
    for attempt in range(10):
        txt = send_sustained(s, b"<0,0,0>", 1.0)
        state = get_state(txt)
        if state == "NONE":
            return state
    return state


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

print("=== ESP LINK INTERACTIVE TEST ===\n")
results = {}

s = open_port()
s.reset_input_buffer()

# Test 1: NO_LINK state
print("TEST 1: Checking initial state...")
txt = read_lines(s, 3)
state = get_state(txt)
ok = state == "NO_LINK"
results["initial_NO_LINK"] = ok
print(f"  State: {state}")
print(f"  {'PASS' if ok else 'FAIL'}: expected NO_LINK\n")

# Establish clean link first
print("  Establishing link...")
state = establish_link(s)
if state != "NONE":
    print(f"  WARNING: state is {state}, not NONE. Bumper may be tripped.")
    print(f"  Clear the area around the ultrasonic sensor.\n")
print(f"  Link state: {state}\n")

# Test 2: Send forward command
print("TEST 2: Sending forward command <80,80,0> for 3s...")
txt = send_sustained(s, b"<80,80,0>", 3.0)
pwm = get_pwm(txt)
state = get_state(txt)
ok = pwm is not None and pwm[0] > 0 and pwm[1] > 0
results["forward_cmd"] = ok
print(f"  State: {state}  PWM: {pwm}")
if not ok and state == "OBSTACLE":
    print(f"  NOTE: bumper tripped. Clear the area and re-run.")
print(f"  {'PASS' if ok else 'FAIL'}: expected positive PWM\n")

# Test 3: Send turn command
print("TEST 3: Sending turn command <-60,60,0> for 3s...")
txt = send_sustained(s, b"<-60,60,0>", 3.0)
pwm = get_pwm(txt)
ok = pwm is not None and pwm[0] < 0 and pwm[1] > 0
results["turn_cmd"] = ok
print(f"  PWM: {pwm}")
print(f"  {'PASS' if ok else 'FAIL'}: expected left<0, right>0\n")

# Test 4: Safety flag stops motors
print("TEST 4: Sending safety stop <0,0,1>...")
txt = send_sustained(s, b"<0,0,1>", 1.5)
pwm = get_pwm(txt)
flag = re.findall(r"flag=(\d)", txt)
ok = pwm == (0, 0) and flag and flag[-1] == "1"
results["safety_flag"] = ok
print(f"  PWM: {pwm}, flag: {flag[-1] if flag else '?'}")
print(f"  {'PASS' if ok else 'FAIL'}: expected PWM=0,0 flag=1\n")

# Test 5: Watchdog timeout
print("TEST 5: Waiting for SIGNAL_LOST (stop sending for 1.5s)...")
time.sleep(1.5)
txt = read_lines(s, 2)
state = get_state(txt)
ok = state == "SIGNAL_LOST"
results["watchdog"] = ok
print(f"  State: {state}")
print(f"  {'PASS' if ok else 'FAIL'}: expected SIGNAL_LOST\n")

# Test 6: Recovery
print("TEST 6: Sending packets to recover...")
txt = send_sustained(s, b"<50,50,0>", 3.0)
state = get_state(txt)
pwm = get_pwm(txt)
ok = state is not None and state != "SIGNAL_LOST" and state != "NO_LINK"
results["recovery"] = ok
print(f"  State: {state}  PWM: {pwm}")
print(f"  {'PASS' if ok else 'FAIL'}: expected recovery from SIGNAL_LOST\n")

# Clean up
s.write(b"<0,0,1>")
s.close()

passed = sum(results.values())
total = len(results)
print(f"=== ESP LINK RESULT: {passed}/{total} passed ===")
summary = []
for name, ok in results.items():
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")
    summary.append(('PASS: %s' % name, (0,255,0)) if ok else ('FAIL: %s' % name, (0,0,255)))
lcd.text([('ESP LINK RESULTS', (255,255,0)), ('', (0,0,0))] + summary)
lcd.close()
