"""NAVI shared configuration.

Every tunable the vision, depth and control layers read lives here so there is
exactly one place to change a threshold, a pin or a timing budget. The values
are not arbitrary. Each one traces back to a numbered section of the Senior
Design 1 Final Report, cited inline, so the code and the document can be checked
against each other.

Nothing in this module imports hardware libraries, so it is safe to import from
a test runner on any machine.
"""

import os

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR = os.path.join(SCRIPT_DIR, "test_images")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")


# ---------------------------------------------------------------------------
# Vision pipeline (report 6.11.1, 6.11.2)
# ---------------------------------------------------------------------------

PROC_WIDTH = 960          # downscale wide frames to this width before processing
MIN_AREA_FRAC = 0.004     # a blob must cover at least 0.4% of the frame to count
# WILLTEST: red needs a bigger blob than green. Across four color_capture.py
# sessions every false red (skin in shadow, tan fabric, stray specks) was at
# most 0.5% of the frame, and every real red marker was at least 3.1% even at
# 1 m. 1% sits between them with margin both ways; a letter size red sheet
# still clears it out to about 4 m. Skin at the knuckles was overriding a green
# target at exactly the 0.4% floor.
RED_MIN_AREA_FRAC = 0.01
MORPH_KERNEL = 5          # size of the open and close structuring element

# Retuned HSV ranges. These were MEASURED with hsv_probe.py against real photos
# rather than guessed, and they are the values printed in report Table 6.11.1.
# Two real detection bugs were found and fixed this way: glare reading as a
# false green target, and a dim red marker falling below the old value floor.
LOWER_GREEN = np.array([35, 50, 40])
UPPER_GREEN = np.array([90, 255, 255])
#
# WILLTEST: red retuned 2026-09-25 from two color_capture.py sessions on the Pi.
# Skin and a tan fabric under warm indoor light (hue 5 to 12, saturation 76 to
# 119) were passing as red hazards. Real red targets measured hue 0 to 3 or
# 172 to 179 with saturation 170 to 205. Narrowing hue to within 6 of pure red
# and raising the saturation floor to 150 removed every false red in both
# sessions and kept every real red target. The value floor stays at 70 so the
# SD1 dim red regression still holds. Report Table 6.11.1 must be updated to
# match; see docs/report-deltas.md.
LOWER_RED_1 = np.array([0, 150, 70])
UPPER_RED_1 = np.array([6, 255, 255])
LOWER_RED_2 = np.array([172, 150, 70])
UPPER_RED_2 = np.array([179, 255, 255])


# ---------------------------------------------------------------------------
# Motor and packet constants (report 6.10, 6.12, 6.13)
# ---------------------------------------------------------------------------

PWM_MAX = 255              # packet PWM range is minus 255 to plus 255
BASE_SPEED = 150           # forward speed when a green target is centered
TURN_GAIN = 1.2            # how hard steering error maps to a turn


# ---------------------------------------------------------------------------
# Camera (report 2.4 Table 2.4.1, 3.7.2, 3.7.5)
# ---------------------------------------------------------------------------

CAMERA_INDEX = 0           # EMEET USB webcam enumerates as /dev/video0 on the Pi
CAMERA_WIDTH = 1280        # 720p is the minimum resolution the report commits to
CAMERA_HEIGHT = 720
CAMERA_FPS = 30            # Table 2.4.1 target
TARGET_FPS = 30            # what the vision thread is judged against

UI_FPS = 30                # the UI redraws at its own cadence, apart from vision


# ---------------------------------------------------------------------------
# Time of Flight depth (report 3.7.4, 3.7.9)
# ---------------------------------------------------------------------------

# Fallback /dev/videoN for the ToF when it cannot be auto detected. On the Pi 5
# with the webcam plugged in it is video2 (the webcam holds video0 and video1).
DEPTH_VIDEO_INDEX = 0

DEPTH_WIDTH = 240          # Arducam ToF depth resolution quoted in 3.7.4
DEPTH_HEIGHT = 180

# Usable measurement window. Section 3.7.4 puts the camera range at roughly 2 to
# 4 m. Readings outside this band are treated as invalid rather than trusted,
# since reflective surfaces and direct sunlight both produce confident looking
# garbage that is worse than no reading at all.
DEPTH_MIN_MM = 200
DEPTH_MAX_MM = 4000

# Zone verdict thresholds. Under BLOCKED_MM the path is refused outright.
# Between that and CAUTION_MM the chair may proceed but is flagged.
ZONE_BLOCKED_MM = 1000
ZONE_CAUTION_MM = 2000

# A zone needs this fraction of its pixels to be valid before its median
# distance is trusted. Below it the zone reports UNKNOWN, which is treated as
# blocked. That is the same fail safe direction as no target detected meaning
# STOP in section 6.11.2.
ZONE_MIN_VALID_FRAC = 0.10

# WILLTEST: each zone is judged by the nearest 5% of its readings, not the
# median. Measured with tof_capture.py: the median missed a hand at 15 to 60 cm
# on every frame, the 5th percentile caught all of them with empty scenes still
# CLEAR. See zone_stats in navi_depth.py.
ZONE_PERCENTILE = 5


# ---------------------------------------------------------------------------
# UART link to the ESP32 (report 6.13)
# ---------------------------------------------------------------------------

# On a Pi 5 the GPIO14 and GPIO15 UART is /dev/ttyAMA0 once enable_uart=1 is set
# and the serial console has been released. See docs/COMMANDS.md for that setup.
#
# WILLTEST: the ESP32 dev board is plugged into the Pi by USB for tonight, so
# the link is the dev board's USB serial bridge rather than the GPIO UART. If it
# shows up as /dev/ttyACM0 instead, pass --port or change this line.
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

# The ESP32 halts if it goes 500 ms without a valid packet (6.13, 6.16). We
# transmit far faster than that so the watchdog never fires in normal operation.
# The margin is what absorbs a scheduling hiccup on the Pi.
UART_TX_INTERVAL_S = 0.05      # 20 Hz
UART_WATCHDOG_MS = 500         # must match WATCHDOG_TIMEOUT_MS in navi_logic.h

# If the vision thread has not produced a fresh command in this long, the UART
# thread stops repeating the stale one and sends a stop instead.
COMMAND_STALE_S = 0.30


# ---------------------------------------------------------------------------
# Metrics (report 6.19)
# ---------------------------------------------------------------------------

PRINT_EVERY = 0.25         # seconds between live status line refreshes
FPS_WINDOW = 30            # frames averaged for the current FPS readout
METRICS_CSV = os.path.join(OUTPUT_DIR, "metrics.csv")


# ---------------------------------------------------------------------------
# ESP32 GPIO map, reference copy
# ---------------------------------------------------------------------------
# The firmware reads these from firmware/main/navi_pins.h. This dict exists so
# the Pi side code and the docs can report the wiring without a second source of
# truth drifting out of sync. If you change one, change both.
#
# See docs/wiring.md for the two non obvious details: the UART TX and RX pins
# are swapped relative to the usual UART2 labelling, and the HC SR04 ECHO lines
# need a divider that the rev 1.0 board does not have.

ESP32_PINS = {
    "UART_TX": 1,          # WILLTEST: UART0 via the dev board USB bridge
    "UART_RX": 3,          # (PCB build uses 16 and 17)
    "ESTOP": 12,           # button normally open contact to GND, active low
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
