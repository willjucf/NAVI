# willtest

A bench-test copy of `../NAVI/` that runs on the hardware we have now:

- webcam on USB
- Arducam ToF on the CAM1 ribbon
- a plain ESP32 dev board plugged into the Pi by USB (no ultrasonics, IMU or e-stop)

It also has fixes from live testing that are not in `../NAVI/` yet.

## Differences from NAVI

| | willtest | NAVI |
|---|---|---|
| ESP32 link | USB, `/dev/ttyUSB0` (UART0) | GPIO UART2, `/dev/ttyAMA0` |
| Webcam | 30 fps | capped at 15 fps (bug) |
| ToF | auto-detected, own process | opens the webcam by mistake, blocks vision |
| Red detection | stricter (fewer false reds on skin) | report values |
| Logging | `--esp-log`, `--log` | none |

## Files

| File | Purpose |
|---|---|
| `navi_config.py` | every tunable setting |
| `navi_vision.py` | webcam, red/green colour detection |
| `navi_depth.py` | ToF depth zones |
| `navi_control.py` | state machine and the USB link to the ESP32 |
| `selftest.py` | hardware check |
| `tools/esp_link_test.py` | ESP32 link test over USB |
| `tools/tof_probe.py`, `tof_capture.py` | ToF readout and guided capture |
| `tools/color_capture.py`, `hsv_probe.py` | colour capture and calibration |
| `tools/pi-wifi-rescue.ps1` | add wifi to the Pi via its SD card (Windows) |
| `firmware/` | ESP32 firmware, USB build |

## Running (on the Pi)

```bash
cd ~/willtest
python3 selftest.py                    # webcam, ToF, ESP32
python3 tools/esp_link_test.py         # ESP32 over USB
python3 navi_control.py --esp-log      # full system
python3 navi_control.py --esp-log --log   # also record CSV + snapshots to output/
```

Keys: `m` MANUAL/AUTO, `e` E-STOP, `r` reset, `q` quit.

## Reflash the ESP32

```bash
. ~/esp/esp-idf/export.sh && cd ~/willtest/firmware && idf.py -p /dev/ttyUSB0 flash
```
