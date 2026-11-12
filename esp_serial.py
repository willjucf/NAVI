"""Open the ESP32's UART through the Vision PCB's CP2102.

Handles two quirks of the link:
1. Opening the port can reset the ESP32 through DTR/RTS. See the order below.
2. Many opens start with up to a second of repeated junk bytes from the CP2102.
   The port is opened once and the junk is drained.
"""

import collections
import glob
import time

BAUD = 115200           # matches SERIAL_BAUD in navi_config

WINDOW_S = 0.5          # the burst is judged in windows this long
MAX_DRAIN_S = 10.0
# 115200 baud 8N1 carries at most 11,520 bytes a second. The ESP32 sends about
# 250 a second (two status lines), plus a few KB of boot log after a reset.
MAX_REAL_BYTES_PER_S = 11520
# The burst is one line over and over. Real lines never repeat exactly: status
# lines carry a timestamp.
MAX_SAME_LINE = 20


def is_flooding(data, seconds):
    """True if data read over seconds could not have come from the ESP32."""
    if len(data) > MAX_REAL_BYTES_PER_S * seconds:
        return True
    lines = [l for l in data.split(b"\n") if l.strip()]
    if not lines:
        return False
    return collections.Counter(lines).most_common(1)[0][1] > MAX_SAME_LINE


def default_port():
    """The CP2102's /dev/serial/by-id path. Doesn't import navi_config, which needs numpy."""
    for pattern in ("/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*CP2102*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return "/dev/ttyUSB0"


def open_esp_serial(port=None, baud=None, timeout=0, write_timeout=None,
                    log=print):
    """Open the CP2102 without resetting the ESP32 and return it once it reads clean.

        If the junk hasn't stopped after MAX_DRAIN_S the port is returned anyway, with a warning.
    """
    import serial

    link = serial.Serial()
    link.port = port or default_port()
    link.baudrate = baud or BAUD
    link.timeout = 0.05
    link.write_timeout = write_timeout
    # Drop RTS before DTR: DTR first would pull EN low and reset the ESP32.
    link.dtr = True
    link.rts = False
    link.open()
    link.dtr = False

    drained = 0
    t_end = time.time() + MAX_DRAIN_S
    while time.time() < t_end:
        data = b""
        t_win = time.time() + WINDOW_S
        while time.time() < t_win:
            data += link.read(8192)
        if not is_flooding(data, WINDOW_S):
            if drained and log:
                log("serial: dropped %d bytes of junk after open" % drained)
            link.timeout = timeout
            return link
        drained += len(data)

    if log:
        log("serial: junk still arriving after %.0f s, using the port anyway. "
            "Packets to the ESP32 get through, its log back may repeat"
            % MAX_DRAIN_S)
    link.timeout = timeout
    return link
