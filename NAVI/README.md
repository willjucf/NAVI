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
| `tools/esp_link_test.py` | Pi to ESP32 link test (12 checks) |
| `tools/bumper_test.py` | watch the ultrasonic bumper trip and release |
| `tools/tof_probe.py`, `tof_capture.py` | ToF readout and guided capture |
| `tools/color_capture.py`, `hsv_probe.py` | colour capture and calibration |
| `firmware/` | ESP32 firmware (ESP-IDF v5.3 or later) |
| `firmware_tests/uss_test/` | standalone ultrasonic test with wiring diagnosis |

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
- Ultrasonic bumper: trips under 300 mm, releases after 500 ms past 400 mm.
- Tilt over 15 degrees halts (once the IMU is answering).
