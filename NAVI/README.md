# NAVI

Software for the NAVI smart wheelchair, UCF Senior Design Group 7.

A Raspberry Pi 5 decides where to go using a webcam and an Arducam ToF depth
camera. An ESP32 decides whether it is safe to, and is the only thing that
drives the motors.

```
EMEET webcam ──USB──┐
                    ├──► Raspberry Pi 5 ──UART 115200──► ESP32
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
| `tools/hsv_probe.py` | colour calibration helper |
| `firmware/` | ESP32 firmware (ESP-IDF) |

## Pi setup

```bash
sudo apt install -y python3-opencv python3-numpy python3-serial
python3 -m pip install --break-system-packages ArducamDepthCamera
sudo raspi-config   # Interface Options > Serial Port: login shell NO, hardware YES
```

## Running

```bash
python3 selftest.py              # check the hardware first
python3 navi_vision.py           # live webcam detection
python3 navi_depth.py            # ToF zone distances
python3 navi_control.py          # full system
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

Unplug the Pi UART link before flashing.

## Wiring notes

- UART is swapped from the usual labels: ESP32 **IO16 is TX**, **IO17 is RX**.
- E-stop on IO12 (to GND), relay IO14, LED IO4, buzzer IO5.
- Ultrasonics: left TRIG/ECHO IO18/IO19, right IO27/IO26.
- BNO055 IMU: SDA IO21, SCL IO22.
- Full pin map is in `firmware/main/navi_pins.h`.

## Safety behaviour

- Red beats green. Nothing detected means stop.
- E-stop latches until an explicit reset, which returns to MANUAL.
- 500 ms watchdogs on both the Pi link and the ESP32 itself.
- The chair boots halted until the ESP32 receives a real packet.
