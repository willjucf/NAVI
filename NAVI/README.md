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
| `tests/` | offline unit tests, no hardware needed |
| `tests-interactive/` | guided hardware tests: webcam, ToF, ultrasonic, ESP link, full system (`test_final.py`) |
| `tools/esp_link_test.py` | Pi to ESP32 link test (12 checks) |
| `tools/bumper_test.py` | watch the ultrasonic bumper trip and release |
| `tools/tof_probe.py`, `tof_capture.py` | ToF readout and guided capture |
| `tools/color_capture.py`, `hsv_probe.py` | colour capture and calibration |
| `tools/cp_diag.py` | CP2102 USB health test, 100 tries per request type (`sudo`, needs `python3-usb`) |
| `tools/pi-wifi-rescue.ps1` | add a wifi network to the Pi from its SD card (Windows) |
| `firmware/` | ESP32 firmware for the final PCB (ESP-IDF v5.3 or later) |
| `firmware/test/` | native unit tests for the firmware logic, run on the Pi with `make` |
| `firmware_tests/uss_test/` | standalone ultrasonic test with wiring diagnosis |
| `firmware_wireless/` | bench build of `firmware/` plus WiFi, Bluetooth and WiFi updates (see below) |

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
  The same build runs on a plain ESP32 dev kit on the Pi's USB.
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

`firmware/` plus the following, for bench work while the Vision board CP2102 is being fixed:

- **WiFi:** joins the iPhone hotspot, then runs a TCP server on port 3333. Use `nc <ip> 3333`.
- **Bluetooth:** BLE Nordic UART Service, advertised as `NAVI-ESP32`. Connect with
  `tools/navi_ble.py` or a phone app such as Bluefruit Connect.
- Both links take the same `<L,R,F>` packets as the UART, with the same watchdog and safety gates.
  They mirror every log line back, and accept the text command `i2cscan`.
- **WiFi updates:** `python3 tools/navi_ota.py build/navi_firmware.bin --ip <ip>` sends an
  update to port 3334, with no cable or buttons.
- It uses a 4 MB flash layout with two update slots (`partitions.csv`). Its first flash has
  to be by cable.

## Test status (2026-10-26)

✅ tested on hardware · 🟡 partly tested · ❌ tested and not working · ⬜ not tested yet

| Part | Status | Evidence |
|---|---|---|
| Unit tests (`tests/`) | ✅ | 129/129 pass on the Pi, 2026-10-26 |
| Firmware logic (`firmware/test/`) | ✅ | 487 checks pass, 2026-10-26 |
| `selftest.py --quick` | ✅ | 11/11, 2026-10-26 |
| Webcam + red/green detection | ✅ | EMEET 1280x720 at 30 fps; red retuned live 2026-10-22 |
| ToF depth zones | ✅ | Replacement Arducam; zones 1 m / 2 m set 2026-10-22 |
| LCD, fullscreen window on X11 | ✅ | 2026-10-22, `DISPLAY=:0`, rotation 270 |
| LCD `--lcd` (raw `/dev/fb0`) | ⬜ | Not used. X11 draws over fb0 while the desktop runs |
| Full system (`tests-interactive/test_final.py`) | ✅ | Auto and manual modes on the ESP dev kit, 2026-10-17 to 10-22 |
| Pi to ESP link, dev kit | ✅ | `esp_link_test.py` 12/12, 2026-10-15 |
| Pi to ESP link, Vision board to MCU PCB | 🟡 | 5/6 on 2026-10-22, then the Vision CP2102 failed (USB -71). The chip is being replaced |
| Flashing with auto-reset through the Vision board | ❌ | DTR/RTS "Protocol error" on the old chip. Retest with the new chip |
| PCB firmware boot on the MCU PCB | ✅ | 2026-10-22, inside the wireless build: E-stop armed IO25, relay IO14, UART0 |
| Drive packets and relay on the MCU PCB | ⬜ | Never tested with packets on the PCB |
| E-stop button, buzzer, LED (aux board) | ⬜ | Aux board not connected yet |
| Ultrasonic, dev kit | ✅ | RCWL-1655 at 3.3 V read 41 to 43 cm, 2026-10-15 (`uss_test`) |
| Ultrasonic on the PCB (U1) | ❌ | Not detected on 2026-10-22. Check 3.3 V supply and cable |
| Bumper at 900 / 1100 mm | 🟡 | Hysteresis logic tested by Shay at 300/400. These values set 2026-10-22, not re-verified on hardware |
| ICM-20948 IMU | ❌ | Nothing answers on I2C. Bus healthy (2026-10-22 scan); suspect nCS / VDDIO. Firmware runs without it |
| Motor PWM (IO32/33) | ⬜ | Placeholder pins, not routed on the PCB |
| Wireless: WiFi join and log mirror | ✅ | 2026-10-22 on the MCU PCB, read from the Mac |
| Wireless: Bluetooth connect | 🟡 | Connected 2026-10-26, but hit a log loop. Fix built, not flashed |
| Wireless: loop fix, WiFi updates, `navi_ota.py` | ⬜ | Built 2026-10-26, needs one cable flash |
| `i2cscan` (boot / command) | ✅ / ⬜ | Boot scan ran 2026-10-22; the text command is not tested |
| `tools/cp_diag.py` | ✅ | Found the Vision CP2102 fault, 2026-10-22 |
| `tools/navi_ble.py` | ✅ | Scanned and connected, 2026-10-26 |
| `tools/bumper_test.py` | ⬜ | Never run |

Bench trees that produced these results are in `../bench-history/`.
