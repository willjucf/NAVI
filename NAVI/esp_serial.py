"""Open the UART to the ESP32 through the Vision PCB's CP2102.

Two quirks of the Vision PCB link are handled here, so every program that talks
to the ESP32 over the CP2102 opens it through this, including navi_control at
boot where nobody is around to restart it.

1. Opening the port could reboot the ESP32. The auto reset (DTR/RTS to EN/IO0)
   saw a pulse on EN every open. See the DTR/RTS order below.

2. Many opens start with a burst of junk: one short piece of the last log line
   ("s=248/2" and the like) repeated for up to a second, faster than 115200 baud
   can carry, so it is not from the ESP32. It stops by itself and the link is
   clean after that, in both directions. Reopening only starts a new burst, so
   the port is opened once and the burst is read off and thrown away.
   Measured 2026-11-11.
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
    """The CP2102 by its stable by-id name, as navi_config finds it.

    Kept here instead of importing navi_config, which needs numpy. The console
    often runs in a shell where ESP-IDF's export.sh put its own Python, without
    numpy, first on the path.
    """
    for pattern in ("/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*CP2102*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return "/dev/ttyUSB0"


def open_esp_serial(port=None, baud=None, timeout=0, write_timeout=None,
                    log=print):
    """Open the CP2102 without resetting the ESP32 and return it once it reads clean.

    Raises the pyserial error if the port cannot be opened. If the junk has not
    stopped after MAX_DRAIN_S, returns the port anyway with a warning, since
    packets to the ESP32 still get through.
    """
    import serial

    link = serial.Serial()
    link.port = port or default_port()
    link.baudrate = baud or BAUD
    link.timeout = 0.05
    link.write_timeout = write_timeout
    # DTR and RTS drive EN and IO0 through the Vision PCB's auto reset. Linux
    # raises both on open and pyserial then sets them one at a time. Dropping
    # DTR first leaves RTS alone for a moment, which pulls EN low and reboots
    # the ESP32. Dropping RTS first only touches IO0 while EN stays high.
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
