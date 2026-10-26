"""NAVI final integration test — auto and manual modes with LCD + live console.

Auto mode: webcam green/red detection + ToF zones + USS bumper → drive decision
Manual mode: arrow keys control PWM directly, sensors still monitor

    python3 test_final.py

Keys:
    m       toggle AUTO/MANUAL
    arrows  manual drive (up/down = speed, left/right = turn)
    space   e-stop
    r       reset from e-stop
    q       quit
"""
import sys, os, time, threading, re

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
os.environ.setdefault("DISPLAY", ":0")

import cv2
import numpy as np
import serial
import navi_config as cfg

# --- Motion constants from wheelchairspeed.png ---
CRUISE_PWM = 139          # ~1.2 m/s estimate (139/255)
MANUAL_MAX_PWM = 150      # cap for bench testing, tune on the chair
CAUTION_PWM = 70          # ~0.6 m/s slow-down
RAMP_PWM_PER_SEC = 75     # 0→cruise (150) in 2 seconds
RAMP_INTERVAL = 0.05      # 20 Hz update
RAMP_STEP = max(int(RAMP_PWM_PER_SEC * RAMP_INTERVAL), 4)  # 4 PWM per tick

# --- ESP serial ---
PORT = cfg.SERIAL_PORT   # the CP2102, found by navi_config
BAUD = 115200

def open_esp():
    s = serial.Serial()
    s.port = PORT; s.baudrate = BAUD; s.timeout = 0.1
    s.dtr = False; s.rts = False; s.open()
    return s

def read_esp_state(ser):
    """Read latest ESP status line, return dict."""
    try:
        data = ser.read(4096).decode(errors="replace")
    except Exception:
        return {}
    lines = [l for l in data.splitlines() if "state=" in l]
    if not lines:
        return {}
    line = lines[-1]
    out = {}
    for m in re.finditer(r"(\w+)=([^\s]+)", line):
        out[m.group(1)] = m.group(2)
    return out

# --- Detection (same as navi_vision) ---

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

# --- ToF ---

def read_tof_zones(depth_src):
    """Return (blocked%, caution%, clear%) or None."""
    if depth_src is None:
        return None
    try:
        frame = depth_src.read()
        if frame is None:
            return None
        valid = frame[(frame >= cfg.DEPTH_MIN_MM) & (frame <= cfg.DEPTH_MAX_MM)]
        if valid.size == 0:
            return None
        total = valid.size
        b = 100 * (valid < cfg.ZONE_BLOCKED_MM).sum() / total
        c = 100 * ((valid >= cfg.ZONE_BLOCKED_MM) & (valid < cfg.ZONE_CAUTION_MM)).sum() / total
        cl = 100 * (valid >= cfg.ZONE_CAUTION_MM).sum() / total
        return b, c, cl
    except Exception:
        return None

# --- Keyboard (non-blocking) ---

class KeyReader:
    def __init__(self):
        import tty, termios
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def poll_all(self):
        """Return list of all pending keys."""
        import select
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                ch2 = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.05)[0] else ''
                if ch2 == '[A': keys.append('up')
                elif ch2 == '[B': keys.append('down')
                elif ch2 == '[C': keys.append('right')
                elif ch2 == '[D': keys.append('left')
            else:
                keys.append(ch)
        return keys

    def restore(self):
        import termios
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# --- Ramp ---

def ramp_toward(current, target, step):
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current

# --- Main ---

def main():
    # Open camera
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    for _ in range(10): cap.read()

    # Open ToF
    depth_src = None
    try:
        from navi_depth import ArducamSource
        depth_src = ArducamSource()
        time.sleep(0.5)
    except Exception as e:
        print("ToF unavailable: %s" % e)

    # Open ESP
    try:
        ser = open_esp()
    except Exception as e:
        print("ESP unavailable: %s" % e)
        ser = None

    # LCD window
    cv2.namedWindow("NAVI", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("NAVI", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    kb = KeyReader()

    mode = "MANUAL"
    estop = False
    manual_fwd = 0
    manual_turn = 0
    current_left = 0
    current_right = 0
    target_left = 0
    target_right = 0
    last_fwd_time = 0.0
    last_turn_time = 0.0
    COAST_TIMEOUT = 0.40  # must be longer than terminal key-release gap (~300ms)
    uss_blocked = False
    uss_val = "-1"
    esp_state = "?"

    print("\033[2J\033[H")  # clear screen
    print("=== NAVI FINAL INTEGRATION TEST ===")
    print("Keys: [m] mode  [arrows] manual  [space] e-stop  [r] reset  [q] quit\n")

    try:
        while True:
            t0 = time.time()

            # --- Input (process all pending keys) ---
            quit_pressed = False
            keys = kb.poll_all()
            has_up = 'up' in keys
            has_down = 'down' in keys
            has_left = 'left' in keys
            has_right = 'right' in keys

            for key in keys:
                if key == 'q':
                    quit_pressed = True
                elif key == 'm':
                    mode = "MANUAL" if mode == "AUTO" else "AUTO"
                    manual_fwd = 0; manual_turn = 0
                elif key == ' ':
                    estop = True
                    manual_fwd = 0; manual_turn = 0
                    current_left = 0; current_right = 0  # instant cut
                elif key == 'r':
                    estop = False
                    mode = "MANUAL"
                    manual_fwd = 0; manual_turn = 0
            if quit_pressed:
                break

            # Arrow keys: joystick-style. Keys set their axis.
            # Only coast to 0 when NO keys at all for COAST_TIMEOUT.
            now = time.time()
            any_arrow = has_up or has_down or has_left or has_right
            if mode == "MANUAL" and not estop:
                if any_arrow:
                    last_fwd_time = now  # any key keeps everything alive

                    # Forward/backward axis — only changes when fwd/back key present
                    if has_up:
                        manual_fwd = MANUAL_MAX_PWM
                    elif has_down:
                        manual_fwd = -MANUAL_MAX_PWM
                    # else: keep current manual_fwd (other axis key is active)

                    # Turn axis — only changes when turn key present
                    if has_left:
                        manual_turn = -80
                        last_turn_time = now
                    elif has_right:
                        manual_turn = 80
                        last_turn_time = now
                    # else: if only fwd/back keys, turn stays until it times out

                    # Turn times out if we're getting fwd keys but no turn keys
                    if not has_left and not has_right and now - last_turn_time > COAST_TIMEOUT:
                        manual_turn = 0

                elif now - last_fwd_time > COAST_TIMEOUT:
                    # No keys at all for COAST_TIMEOUT — full stop
                    manual_fwd = 0
                    manual_turn = 0

            # --- Sensors ---
            ok, frame = cap.read()
            if not ok:
                continue

            green, red, gc, rc = detect(frame)
            tof = read_tof_zones(depth_src)
            tof_blocked = tof[0] > 15 if tof else False
            tof_caution = tof[1] > 10 if tof else False

            # Read ESP — keep last known values if no new data
            esp = read_esp_state(ser) if ser else {}
            if "uss" in esp:
                uss_val = esp["uss"]
            if "state" in esp:
                esp_state = esp["state"]
            try:
                uss_mm = int(''.join(c for c in uss_val if c.isdigit() or c == '-'))
            except ValueError:
                uss_mm = -1
            # USS blocked latches — only clears on a confirmed clear reading
            if 0 < uss_mm < 900:
                uss_blocked = True
            elif uss_mm > 1100 or uss_mm == -1:
                uss_blocked = False
            # 900-1100: hold current state (hysteresis)

            # --- Decision ---
            if estop:
                reason = "E-STOP"
                target_left = 0; target_right = 0
                status_color = (0, 0, 255)
            elif mode == "MANUAL":
                reason = "MANUAL"
                target_left = manual_fwd + manual_turn
                target_right = manual_fwd - manual_turn
                status_color = (255, 200, 0)
            elif uss_blocked:
                reason = "USS BLOCKED (%smm)" % uss_val
                target_left = 0; target_right = 0
                status_color = (0, 0, 255)
            elif esp_state == "TILT":
                reason = "TILT DETECTED"
                target_left = 0; target_right = 0
                status_color = (0, 0, 255)
            elif mode == "AUTO":
                if red:
                    reason = "RED STOP"
                    target_left = 0; target_right = 0
                    status_color = (0, 0, 255)
                elif tof_blocked:
                    reason = "TOF BLOCKED (%.0f%%)" % tof[0]
                    target_left = 0; target_right = 0
                    status_color = (0, 0, 255)
                elif tof_caution and green:
                    reason = "CAUTION SLOW (%.0f%%)" % tof[1]
                    speed = CAUTION_PWM
                    if gc is not None:
                        M = cv2.moments(gc)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            err = (cx - frame.shape[1] // 2) / (frame.shape[1] // 2)
                            turn = int(err * speed * cfg.TURN_GAIN)
                            target_left = speed + turn
                            target_right = speed - turn
                    status_color = (0, 165, 255)
                elif green:
                    reason = "GO GREEN"
                    speed = CRUISE_PWM
                    if gc is not None:
                        M = cv2.moments(gc)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            err = (cx - frame.shape[1] // 2) / (frame.shape[1] // 2)
                            turn = int(err * speed * cfg.TURN_GAIN)
                            target_left = speed + turn
                            target_right = speed - turn
                    status_color = (0, 255, 0)
                else:
                    reason = "NO TARGET"
                    target_left = 0; target_right = 0
                    status_color = (200, 200, 200)
            else:
                reason = "UNKNOWN"
                target_left = 0; target_right = 0
                status_color = (100, 100, 100)

            # --- Ramp (e-stop is instant, everything else ramps) ---
            if estop or reason == "E-STOP":
                current_left = 0; current_right = 0
            else:
                if target_left == target_right:
                    # Straight: lock wheels together, ramp as one unit.
                    # Pick the wheel closer to the target so we never
                    # overshoot or go the wrong direction.
                    dl = abs(current_left - target_left)
                    dr = abs(current_right - target_right)
                    base = current_left if dl <= dr else current_right
                    val = ramp_toward(base, target_left, RAMP_STEP)
                    current_left = val
                    current_right = val
                else:
                    current_left = ramp_toward(current_left, target_left, RAMP_STEP)
                    current_right = ramp_toward(current_right, target_right, RAMP_STEP)

            # Clamp
            current_left = max(-MANUAL_MAX_PWM, min(MANUAL_MAX_PWM, current_left))
            current_right = max(-MANUAL_MAX_PWM, min(MANUAL_MAX_PWM, current_right))

            # Send to ESP
            flag = 1 if (estop or (mode == "AUTO" and (red or tof_blocked))) else 0
            if ser:
                pkt = "<%d,%d,%d>" % (int(current_left), int(current_right), flag)
                try:
                    ser.write(pkt.encode())
                except Exception:
                    pass

            # --- LCD display ---
            disp = frame.copy()
            if rc is not None:
                x, y, bw, bh = cv2.boundingRect(rc)
                cv2.rectangle(disp, (x, y), (x+bw, y+bh), (0, 0, 255), 3)
            if gc is not None:
                x, y, bw, bh = cv2.boundingRect(gc)
                cv2.rectangle(disp, (x, y), (x+bw, y+bh), (0, 255, 0), 3)

            # Status bar
            cv2.rectangle(disp, (0, 0), (disp.shape[1], 60), (0, 0, 0), -1)
            cv2.putText(disp, "[%s] %s" % (mode, reason), (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(disp, "PWM L:%d R:%d | USS:%s" % (int(current_left), int(current_right), uss_val),
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            lcd_frame = cv2.resize(disp, (480, 320))
            cv2.imshow("NAVI", lcd_frame)
            cv2.waitKey(1)

            # --- Console (live update, fixed position) ---
            tof_str = "B:%.0f%% C:%.0f%% CL:%.0f%%" % tof if tof else "n/a"
            grn = "YES" if green else "no"
            rd = "YES" if red else "no"

            sys.stdout.write("\033[6;1H\033[J")  # move to line 6, clear below
            sys.stdout.write("  Mode:    %-8s  |  Reason: %s\n" % (mode, reason))
            sys.stdout.write("  PWM:     L:%-4d R:%-4d  (target L:%-4d R:%-4d)\n" % (
                int(current_left), int(current_right), target_left, target_right))
            sys.stdout.write("  Green:   %-3s  |  Red: %-3s\n" % (grn, rd))
            sys.stdout.write("  ToF:     %s\n" % tof_str)
            sys.stdout.write("  USS:     %s mm  |  ESP: %s\n" % (uss_val, esp_state))
            sys.stdout.write("  E-Stop:  %s\n" % ("ACTIVE" if estop else "clear"))
            sys.stdout.flush()

            # Pace at 20 Hz
            elapsed = time.time() - t0
            if elapsed < RAMP_INTERVAL:
                time.sleep(RAMP_INTERVAL - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors
        if ser:
            ser.write(b"<0,0,1>")
            ser.close()
        kb.restore()
        cap.release()
        if depth_src:
            depth_src.close()
        cv2.destroyAllWindows()
        print("\033[13;1H\n=== Test ended ===")

if __name__ == "__main__":
    main()
