# NAVI

Software for the NAVI smart wheelchair, UCF Senior Design Group 7.

A Raspberry Pi 5 decides where to go using a webcam and an Arducam ToF depth
camera. An ESP32 decides whether it is safe to, and is the only thing that
drives the motors.

```
EMEET webcam ──USB──┐
                    ├──► Raspberry Pi 5 ──USB──► CP2102 ──UART0 115200──► ESP32
Arducam ToF ─ribbon─┘
```

Packets from the Pi are `<LEFT_PWM,RIGHT_PWM,SAFETY_FLAG>`, PWM from -255 to 255.

## Files

| File | Purpose |
|---|---|
| `navi_config.py` | every tunable setting in one place |
| `navi_vision.py` | webcam, red/green colour detection |
| `navi_depth.py` | ToF depth, split into left/center/right zones |
| `navi_control.py` | state machine (MANUAL/AUTO/ESTOP) and the UART link |
| `selftest.py` | hardware check, run before a live test |
| `esp_wireless.py` | bench testing only: `navi_control.py --link wifi/ble/both` sends packets without the UART |
| `tests/` | offline unit tests, no hardware needed |
| `tests-interactive/` | guided hardware tests: webcam, ToF, ultrasonic, ESP link, full system (`test_final.py`) |
| `tools/esp_link_test.py` | Pi to ESP32 link test (12 checks) |
| `tools/bumper_test.py` | watch the ultrasonic bumper trip and release |
| `tools/tof_probe.py`, `tof_capture.py` | ToF readout and guided capture |
| `tools/color_capture.py`, `hsv_probe.py` | colour capture and calibration |
| `tools/navi_console.py` | readable live ESP console over the UART, WiFi or Bluetooth: fixed status panel, event log, command line |
| `tools/cp_diag.py` | CP2102 USB health test, 100 tries per request type (`sudo`, needs `python3-usb`) |
| `tools/pi-wifi-rescue.ps1` | add a wifi network to the Pi from its SD card (Windows) |
| `firmware/` | ESP32 firmware for the final PCB (ESP-IDF v5.3 or later) |
| `firmware/test/` | native unit tests for the firmware logic, run on the Pi with `make` |
| `firmware_tests/uss_test/` | standalone ultrasonic test with wiring diagnosis |
| `firmware_wireless/` | `firmware/` plus WiFi, Bluetooth and WiFi updates, for bench work (see below) |

## Pi setup

```bash
sudo apt install -y python3-opencv python3-numpy python3-serial
python3 -m pip install --break-system-packages ArducamDepthCamera
sudo usermod -aG dialout $USER   # serial access, log out and back in
```

## Running

```bash
python3 selftest.py              # check the hardware first
python3 navi_vision.py           # live webcam detection
python3 navi_depth.py            # ToF zone distances
python3 tools/esp_link_test.py   # Pi to ESP32 link
python3 navi_control.py --esp-log          # full system, with ESP32 status
python3 navi_control.py --esp-log --log    # also record CSV + snapshots
```

Keys in `navi_control.py`: `m` MANUAL/AUTO, `e` E-STOP, `r` reset, `q` quit.

Useful flags when hardware is missing: `--headless`, `--sim` (depth),
`--no-serial`, `--no-depth`. Everything takes `--help`.

## Firmware

```bash
. ~/esp/esp-idf/export.sh
cd firmware && idf.py set-target esp32 && idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Stop `navi_control.py` first, since it holds the port. Flashing uses the
CP2102's auto reset. If it does not connect, hold BOOT, tap RESET, release BOOT.

## Wiring notes (final PCB)

- Pi link: UART0 (TX IO1, RX IO3) through the CP2102 board, `/dev/ttyUSB0`.
- E-stop on IO25 (to GND), relay IO14, LED IO4, buzzer IO5.
- Ultrasonic on header U1, IO26/IO27. The firmware detects which pin is
  trigger and which is echo. Echo must not see 5 V: divider, or 3.3 V supply.
- ICM-20948 IMU: SDA IO21, SCL IO22, address 0x68 or 0x69 (both tried).
- Motor PWM IO32/IO33 are placeholders, not routed on the PCB.
- Full pin map is in `firmware/main/navi_pins.h`.

## Safety behaviour

- Red beats green. Nothing detected means stop.
- E-stop latches until an explicit reset, which returns to MANUAL.
- 500 ms watchdogs on both the Pi link and the ESP32 itself.
- The chair boots halted until the ESP32 receives a real packet.
- Ultrasonic bumper: trips under 900 mm, releases after 500 ms past 1100 mm.
- Tilt over 15 degrees halts (once the IMU is answering).

## Wireless bench build (`firmware_wireless/`)

`firmware/` plus the following, for bench work without a cable:

- **WiFi:** joins the hotspot, then serves the log and takes packets on TCP port 3333.
- **Bluetooth:** advertised as `NAVI-ESP32`, with a Nordic UART service (log and packets) and a
  firmware update service.
- Both links take the same `<L,R,F>` packets as the UART, with the same watchdog and safety gates,
  and mirror every log line back. Each takes one client at a time.
- **Text commands** on either link: `i2cscan` (I2C bus check) and `ussdiag` (ultrasonic header
  check: idle level of each pin, and whether echo moves after a trigger).
- **Firmware updates without a cable** (from `firmware_wireless/tools/`):
  `python3 navi_ota.py <app.bin> --ip <esp-ip>` over WiFi (~11 s), or
  `.venv/bin/python navi_ota.py <app.bin> --ble` over Bluetooth (~90 s). A failed or cut-off
  update leaves the running firmware untouched.
- It uses a 4 MB flash layout with two update slots (`partitions.csv`). The first flash of this
  layout has to be by cable.
- WiFi credentials go in `main/wifi_credentials.h` (gitignored; copy the `.example.h`).

Running the Pi without the UART, for testing: `python3 navi_control.py --link wifi` (or `ble`,
or `both`). Watch with `tools/navi_console.py` on the other link, since each takes one client.

## Test status (2026-10-26)

Status of each part on the final PCB system.
✅ working · 🟡 partly working · ❌ not working · ⬜ not tested yet

| Part | Status | Notes |
|---|---|---|
| Unit tests (`tests/`) | ✅ | 129/129 on the Pi |
| Firmware logic (`firmware/test/`) | ✅ | 487 checks |
| `selftest.py --quick` | ✅ | 11/11 |
| Webcam + red/green detection | ✅ | EMEET on `/dev/video0` |
| ToF depth zones | ✅ | ~33 fps, 99% valid pixels; blocked under 1 m, caution 1 to 2 m |
| LCD (fullscreen window) | ✅ | 3.5" ILI9486 on X11, `DISPLAY=:0`, rotation 270 |
| LCD `--lcd` (raw `/dev/fb0`) | ⬜ | X11 draws over fb0 while the desktop runs |
| Full system, `navi_control.py` to the MCU PCB | ✅ | 101,317 frames (9.1 ms mean), 4,323 packets, 0 link errors, AUTO mode |
| `tests-interactive/test_final.py` on PCBs | ⬜ | |
| Pi to ESP over the Vision PCB (CP2102) | ❌ | The CP2102 on the Vision PCB does not power up (VDD 0 V); with the hardware team |
| Pi to ESP over the Pi's UART pins (`/dev/ttyAMA0`) | ✅ | Both directions; Pi pin 8 → P3 pin 5, pin 10 ← P3 pin 4, pin 6 GND |
| Flashing by cable (Pi UART, BOOT + EN) | ✅ | |
| Flashing with auto-reset (DTR/RTS) | ⬜ | Needs a working Vision PCB |
| Firmware update over WiFi / Bluetooth | ✅ / ✅ | 964 KB in ~11 s / ~90 s, reboots into the new image |
| ESP firmware on the MCU PCB | ✅ | E-stop armed on IO25, relay on IO14; arms on packets, `SIGNAL_LOST` 0.5 s after they stop |
| E-stop button, buzzer, LED (aux board) | ⬜ | Aux board not connected yet |
| Ultrasonic (U1, IO26/27) | ❌ | `ussdiag`: both pins float, so the sensor's Echo does not reach the ESP. Echo connection open; Trig and 5 V are fine |
| Bumper at 900 / 1100 mm | 🟡 | Trip/release logic tested; these distances not yet checked on hardware |
| ICM-20948 IMU | ❌ | No answer at 0x68/0x69. A device now answers at 0x10 (unknown). Tilt is ignored without it |
| Motor PWM (IO32/33) | ⬜ | Placeholder pins, not routed on the PCB |
| Wireless links: WiFi / Bluetooth | ✅ / ✅ | Log mirror, packets and commands on both |
| `navi_control.py --link ble` / `wifi` / `both` | ✅ / ⬜ / ⬜ | BLE: 60/60 packets, arms and stops correctly |
| `tools/navi_console.py` | ✅ | |
| `tools/cp_diag.py`, `tools/navi_ble.py` | ✅ | |
| `tools/bumper_test.py` | ⬜ | |
