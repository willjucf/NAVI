# NAVI

Assisted driving for a powered wheelchair. The rider drives with the joystick.
NAVI watches for hazards and cuts the joystick through 4 relays when one appears.

- **Raspberry Pi 5** (Python): webcam red/green detection, Arducam ToF, touchscreen UI.
- **ESP32** (C++, ESP-IDF): E-stop, ultrasonic bumper, IMU tilt, relays.
- Link: Pi USB → CP2102 → ESP32 UART0, 115200 baud, `<L,R,F>` packets at 20 Hz.
  `F=1` cuts the joystick, `F=0` clears it.

## What cuts the joystick

| Condition | Checked on | Clears |
|---|---|---|
| E-stop pressed, cut or unplugged (normally closed) | ESP32 | released for 100 ms |
| No valid packet for 500 ms | ESP32 | packets again |
| Ultrasonic under 1540 mm | ESP32 | past 1740 mm for 0.5 s |
| Tilt over 15° | ESP32 | back under 15° |
| Red in view | Pi | gone for 0.5 s |
| ToF center under 1540 mm, or no reading | Pi | past 1740 mm, then 0.5 s |
| Camera or ToF silent for 0.5 s | Pi | data again, then 0.5 s |
| STOP on the screen | Pi | RESUME |

The 1540 mm stop distance assumes 1100 mm of braking at 2.2 m/s, which hasn't been
measured yet. Set `BRAKE_DISTANCE_MM` in `navi_config.py` and `BUMPER_STOP_MM` in
`firmware/main/sensors.h` together.

## Layout

```
navi_config.py      every threshold, pin and timing
navi_control.py     UART thread and state machine
navi_vision.py      webcam capture and red/green detection
navi_depth.py       ToF zones
esp_serial.py       opens the CP2102 without resetting the ESP32
esp_wireless.py     WiFi/BLE link to the ESP32 (bench use)
selftest.py         preflight checks
start_navi.sh       launcher, restarts on crash
firmware/           ESP32 firmware (ESP-IDF)
tools/              console, calibration and measurement scripts
tests/              unit tests, firmware logic tests, hardware tests
NAVI-full/          full system with AUTO steering (needs motor PWM)
```

## Run

```bash
python3 navi_app.py                 # fullscreen on the LCD
python3 navi_app.py --window        # normal window
python3 navi_app.py --no-serial     # without the ESP32
./install.sh                        # icon + start at boot (--no-autostart for icon only)
tail -f output/navi.log
```

The app opens OFF. START opens the camera, ToF and ESP32 link. The joystick only
works while NAVI is running. With no packets the ESP32 keeps the relays open.

Needs: `python3-opencv`, `python3-numpy`, `python3-serial`, Arducam ToF SDK.

## Firmware

```bash
cp firmware/main/wifi_credentials.example.h firmware/main/wifi_credentials.h   # once
. ~/esp/esp-idf/export.sh
cd firmware && idf.py build
cd build && esptool.py --chip esp32 -p /dev/ttyUSB0 -b 115200 write_flash @flash_args
```

If it won't connect: hold BOOT, tap EN, release BOOT, and add `--before no_reset`.

Wireless update, no cable:

```bash
python3 firmware/tools/navi_ota.py firmware/build/navi_firmware.bin --ip <esp-ip>
```

Console over serial, WiFi (`--ip`) or Bluetooth (`--ble`):

```bash
python3 tools/navi_console.py
```

Commands: `beep`, `i2cscan`, `ussdiag`.

## Tests

```bash
python3 -m unittest discover -s tests       # Pi code, no hardware
cd tests/firmware && make run               # firmware logic, host g++
python3 selftest.py                         # preflight on the Pi
```

`tests/hardware/` are interactive tests for the real webcam, ToF, ultrasonic and link.
