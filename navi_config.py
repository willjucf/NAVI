"""NAVI configuration: every threshold, pin and timing in one place. No hardware imports."""

import os

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR = os.path.join(SCRIPT_DIR, "test_images")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")


# Vision pipeline

PROC_WIDTH = 960          # wide frames are downscaled to this before processing
MIN_AREA_FRAC = 0.004     # blob must cover 0.4% of the frame to count

# Red needs a bigger blob than green. False reds (skin, fabric) measured under 0.5%,
# real markers over 3.1%.
RED_MIN_AREA_FRAC = 0.01

MORPH_KERNEL = 5          # open and close kernel size

# HSV ranges measured with tools/hsv_probe.py.
LOWER_GREEN = np.array([35, 50, 40])
UPPER_GREEN = np.array([90, 255, 255])

# Red tuned on the live webcam: skin measures sat 80-105, so a floor of 140 rejects it.
LOWER_RED_1 = np.array([0, 140, 70])
UPPER_RED_1 = np.array([7, 255, 255])
LOWER_RED_2 = np.array([160, 140, 70])
UPPER_RED_2 = np.array([179, 255, 255])


# Motor and packet constants

PWM_MAX = 255              # packet PWM range is minus 255 to plus 255
BASE_SPEED = 150           # forward speed with a centered green target
# Steering gain. Raise if the target drifts out of view, lower if the chair oscillates.
TURN_GAIN = 0.5


# Camera

CAMERA_INDEX = 0           # EMEET webcam is /dev/video0 on the Pi
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30
TARGET_FPS = 30            # what the vision thread is judged against

UI_FPS = 30                # UI redraws on its own clock, separate from vision


# Time of Flight depth

# Fallback /dev/videoN for the ToF if auto detect fails. With the webcam in it
# is video2, since the webcam takes video0 and video1.
DEPTH_VIDEO_INDEX = 2

DEPTH_WIDTH = 240          # Arducam ToF resolution
DEPTH_HEIGHT = 180

# Zero (no return) and readings past DEPTH_MAX_MM are dropped. Close readings are kept.
DEPTH_MIN_MM = 200
DEPTH_MAX_MM = 4000

# Below this confidence the Arducam reports filler values (500, 1000, 2000 mm).
DEPTH_MIN_CONFIDENCE = 30

# Stop distance at top speed. Must match BUMPER_STOP_MM in firmware/main/sensors.h.
MAX_SPEED_MPS = 2.2
# Detection to relays open: one camera/ToF frame (33 ms), the next packet
# (50 ms), the ESP32's 10 ms loop and the relay itself, rounded up.
REACTION_S = 0.2
# Rolling distance after the relays open at MAX_SPEED_MPS. Not measured yet (assumes ~1 s).
BRAKE_DISTANCE_MM = 1100
STOP_DISTANCE_MM = int(MAX_SPEED_MPS * REACTION_S * 1000) + BRAKE_DISTANCE_MM  # 1540
# Once tripped, the ToF has to read past this before it counts as clear, so an
# object right at the limit can't chatter the relays.
STOP_RELEASE_MM = STOP_DISTANCE_MM + 200
# Every hazard has to be gone this long before the relays close again.
HAZARD_CLEAR_S = 0.5
# The camera or the ToF silent this long counts as a hazard: blind is not clear.
SENSOR_STALE_S = 0.5

# Zone verdicts. Under BLOCKED_MM the path is refused. Between that and
# CAUTION_MM it is flagged on the screen.
ZONE_BLOCKED_MM = STOP_DISTANCE_MM
ZONE_CAUTION_MM = 2000

# Fraction of valid pixels a zone needs. Below it the zone is UNKNOWN and counts as blocked.
ZONE_MIN_VALID_FRAC = 0.10

# Zones use the 5th percentile, not the median, so a hand close to the lens isn't missed.
ZONE_PERCENTILE = 5


# UART link to the ESP32

# Pi USB -> CP2102 (Vision PCB) -> ESP32 UART0. --port overrides.
def _find_serial_port():
    import glob
    for pattern in ("/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*CP2102*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return "/dev/ttyUSB0"


SERIAL_PORT = _find_serial_port()
SERIAL_BAUD = 115200

# The ESP32 halts after 500 ms without a valid packet. We send
# much faster so the margin absorbs a scheduling hiccup on the Pi.
UART_TX_INTERVAL_S = 0.05      # 20 Hz
UART_WATCHDOG_MS = 500         # must match WATCHDOG_TIMEOUT_MS in navi_logic.h

# If vision has not produced a new command in this long, the UART thread sends
# a stop instead of repeating the stale one.
COMMAND_STALE_S = 0.30


# Metrics

PRINT_EVERY = 0.25         # seconds between status line refreshes
FPS_WINDOW = 30            # frames averaged for the FPS readout
METRICS_CSV = os.path.join(OUTPUT_DIR, "metrics.csv")


# ESP32 pins, mirrored from firmware/main/navi_pins.h. selftest.py checks they match.

ESP32_PINS = {
    "UART_TX": 1,          # UART0 TXD0, through the CP2102
    "UART_RX": 3,          # UART0 RXD0
    "ESTOP": 25,           # normally closed button to GND (P2 pin 1)
    "RELAY": 14,           # motor cutoff relay
    "LED": 4,
    "BUZZER": 5,
    "USS_A": 26,           # ultrasonic header U1 pin 2; the firmware works
    "USS_B": 27,           # out which is trigger and which is echo
    "I2C_SDA": 21,         # onboard ICM-20948
    "I2C_SCL": 22,
}
