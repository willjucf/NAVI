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

# WILLTEST: red needs a bigger blob than green. Across four color_capture.py
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

# WILLTEST: red retuned 2026-09-25 from two color_capture.py sessions. Skin and
# tan fabric under warm light (hue 5 to 12, sat 76 to 119) passed as red. Real
# red measured hue 0 to 3 or 172 to 179, sat 170 to 205. Hue within 6 of pure
# red and a sat floor of 150 removed every false red and kept every real one.
# Value floor stays 70 so the dim red regression still passes. Report Table
# 6.11.1 needs updating.
LOWER_RED_1 = np.array([0, 150, 70])
UPPER_RED_1 = np.array([6, 255, 255])
LOWER_RED_2 = np.array([172, 150, 70])
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
# CAUTION_MM the chair may go but it is flagged.
ZONE_BLOCKED_MM = 600
ZONE_CAUTION_MM = 1200

# A zone needs this fraction of valid pixels to be trusted. Below it the zone
# is UNKNOWN, which counts as blocked. Same fail safe idea as no target meaning
# STOP (6.11.2).
ZONE_MIN_VALID_FRAC = 0.10

# WILLTEST: each zone is judged by its nearest 5% of readings, not the median.
# With tof_capture.py the median missed a hand at 15 to 60 cm on every frame.
# The 5th percentile caught all of them and empty scenes stayed CLEAR. See
# zone_stats in navi_depth.py.
ZONE_PERCENTILE = 5


# UART link to the ESP32 (report 6.13)

# On a Pi 5 the GPIO14/15 UART is /dev/ttyAMA0 once enable_uart=1 is set and the
# serial console is released. Setup is in README.md.
#
# WILLTEST: the dev board is on the Pi's USB, so the link is its USB serial
# bridge, not the GPIO UART. If it shows up as /dev/ttyACM0, pass --port or
# change this line.
SERIAL_PORT = "/dev/ttyUSB0"
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
# Two gotchas, see README.md: UART TX and RX are swapped from the usual
# UART2 labels, and the HC SR04 ECHO lines need a divider rev 1.0 lacks.

ESP32_PINS = {
    "UART_TX": 1,          # WILLTEST: UART0 through the dev board USB bridge
    "UART_RX": 3,          # (PCB build uses 16 and 17)
    "ESTOP": 12,           # button to GND, active low
    "RELAY": 14,           # motor cutoff relay (report 3.9.5)
    "LED": 4,
    "BUZZER": 5,
    "USS1_TRIG": 18,
    "USS1_ECHO": 19,
    "USS2_TRIG": 27,
    "USS2_ECHO": 26,
    "I2C_SDA": 21,         # BNO055
    "I2C_SCL": 22,
}
