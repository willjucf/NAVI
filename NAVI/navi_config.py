"""NAVI shared configuration.

Every tunable for vision, depth and control lives here so there is one place to
change a threshold, pin or timing. Each value cites its SD1 Final Report section
so code and report can be checked against each other.

No hardware imports, so tests can import this on any machine.
"""

import os

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR = os.path.join(SCRIPT_DIR, "test_images")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")


# Vision pipeline (report 6.11.1, 6.11.2)

PROC_WIDTH = 960          # wide frames are downscaled to this before processing
MIN_AREA_FRAC = 0.004     # blob must cover 0.4% of the frame to count

# Red needs a bigger blob than green. Across four color_capture.py
# sessions every false red (shadowed skin, tan fabric, specks) was at most 0.5%
# of the frame and every real red marker was at least 3.1%. 1% sits between
# with margin, and a letter size red sheet still clears it to about 4 m.
# Knuckle skin at the 0.4% floor was overriding green targets.
RED_MIN_AREA_FRAC = 0.01

MORPH_KERNEL = 5          # open and close kernel size

# HSV ranges measured with hsv_probe.py on real photos, not guessed. These are
# the values in report Table 6.11.1. Measuring fixed two real bugs: glare
# reading as a false green, and a dim red marker under the old value floor.
LOWER_GREEN = np.array([35, 50, 40])
UPPER_GREEN = np.array([90, 255, 255])

# Red retuned 2026-09-25 from two color_capture.py sessions. Skin and
# tan fabric under warm light (hue 5 to 12, sat 76 to 119) passed as red. Real
# red measured hue 0 to 3 or 172 to 179, sat 170 to 205. Hue within 6 of pure
# red and a sat floor of 150 removed every false red and kept every real one.
# Value floor stays 70 so the dim red regression still passes. Report Table
# 6.11.1 needs updating.
#
# Widened again 2026-10-22 on the live webcam: hue 0 to 7 and 160 to 179, sat
# floor 140. Skin measured sat 80 to 105, so 140 still rejects it, and a floor
# of 210 missed red targets at a distance.
LOWER_RED_1 = np.array([0, 140, 70])
UPPER_RED_1 = np.array([7, 255, 255])
LOWER_RED_2 = np.array([160, 140, 70])
UPPER_RED_2 = np.array([179, 255, 255])


# Motor and packet constants (report 6.10, 6.12, 6.13)

PWM_MAX = 255              # packet PWM range is minus 255 to plus 255
BASE_SPEED = 150           # forward speed with a centered green target
TURN_GAIN = 1.2            # how hard steering error turns the chair


# Camera (report 2.4 Table 2.4.1, 3.7.2, 3.7.5)

CAMERA_INDEX = 0           # EMEET webcam is /dev/video0 on the Pi
CAMERA_WIDTH = 1280        # 720p is the minimum the report commits to
CAMERA_HEIGHT = 720
CAMERA_FPS = 30            # Table 2.4.1 target
TARGET_FPS = 30            # what the vision thread is judged against

UI_FPS = 30                # UI redraws on its own clock, separate from vision


# Time of Flight depth (report 3.7.4, 3.7.9)

# Fallback /dev/videoN for the ToF if auto detect fails. With the webcam in it
# is video2, since the webcam takes video0 and video1.
DEPTH_VIDEO_INDEX = 2

DEPTH_WIDTH = 240          # Arducam ToF resolution from 3.7.4
DEPTH_HEIGHT = 180

# Usable range (3.7.4 says about 2 to 4 m). Readings outside it are invalid.
# Shiny surfaces and sunlight give confident looking garbage, which is worse
# than no reading.
DEPTH_MIN_MM = 200
DEPTH_MAX_MM = 4000

# Zone verdicts. Under BLOCKED_MM the path is refused. Between that and
# CAUTION_MM the chair may go but it is flagged. Raised 2026-10-22 on the
# replacement ToF: blocked under 1 m (39 in), caution from 1 to 2 m.
ZONE_BLOCKED_MM = 1000
ZONE_CAUTION_MM = 2000

# A zone needs this fraction of valid pixels to be trusted. Below it the zone
# is UNKNOWN, which counts as blocked. Same fail safe idea as no target meaning
# STOP (6.11.2).
ZONE_MIN_VALID_FRAC = 0.10

# Each zone is judged by its nearest 5% of readings, not the median.
# With tof_capture.py the median missed a hand at 15 to 60 cm on every frame.
# The 5th percentile caught all of them and empty scenes stayed CLEAR. See
# zone_stats in navi_depth.py.
ZONE_PERCENTILE = 5


# UART link to the ESP32 (report 6.13)

# Pi USB -> CP2102 (Vision PCB, or the dev kit's own bridge) -> ESP32 UART0.
# The by-id name survives replugging and reboots, so prefer it. Falls back to
# /dev/ttyUSB0, and --port overrides both.
def _find_serial_port():
    import glob
    for pattern in ("/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*CP2102*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return "/dev/ttyUSB0"


SERIAL_PORT = _find_serial_port()
SERIAL_BAUD = 115200

# The ESP32 halts after 500 ms without a valid packet (6.13, 6.16). We send
# much faster so the margin absorbs a scheduling hiccup on the Pi.
UART_TX_INTERVAL_S = 0.05      # 20 Hz
UART_WATCHDOG_MS = 500         # must match WATCHDOG_TIMEOUT_MS in navi_logic.h

# If vision has not produced a new command in this long, the UART thread sends
# a stop instead of repeating the stale one.
COMMAND_STALE_S = 0.30


# Metrics (report 6.19)

PRINT_EVERY = 0.25         # seconds between status line refreshes
FPS_WINDOW = 30            # frames averaged for the FPS readout
METRICS_CSV = os.path.join(OUTPUT_DIR, "metrics.csv")


# ESP32 GPIO map, reference copy
#
# The firmware uses firmware/main/navi_pins.h. This copy lets the Pi side and
# docs report the wiring. Change one, change both.
#
# Version C, the final MCU PCB. See docs/NAVI/pin-versions.md.

ESP32_PINS = {
    "UART_TX": 1,          # UART0 TXD0, through the CP2102
    "UART_RX": 3,          # UART0 RXD0
    "ESTOP": 25,           # button to GND, active low (P2 pin 1)
    "RELAY": 14,           # motor cutoff relay (report 3.9.5)
    "LED": 4,
    "BUZZER": 5,
    "USS_A": 26,           # ultrasonic header U1 pin 2; the firmware works
    "USS_B": 27,           # out which is trigger and which is echo
    "I2C_SDA": 21,         # onboard ICM-20948
    "I2C_SCL": 22,
}
