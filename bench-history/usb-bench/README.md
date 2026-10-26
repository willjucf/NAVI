# willtest: NAVI bench-test tree

A copy of `../NAVI/` adapted to run **today** on the real hardware:

- webcam on USB
- Arducam ToF on the CAM1 ribbon
- a **plain ESP32 dev board plugged into the Pi by USB**, with no GPIO wiring, ultrasonics, IMU or e-stop

It also carries the fixes and retuning found during live testing on 2026-09-25, none of
which are merged into `../NAVI/` yet. `docs/report-deltas.md` lists each one and the report
section it affects.

On the Pi this folder lives at `~/willtest`, identical to this copy.

## Differences from `../NAVI/`

| Area | willtest | NAVI (PCB) |
|---|---|---|
| ESP32 link | USB bridge, UART0 (GPIO1/3), `/dev/ttyUSB0` | GPIO UART2 (IO16/IO17), `/dev/ttyAMA0` |
| Webcam | 2 capture buffers, 30 fps | 1 buffer, capped at 15 fps (bug) |
| ToF | Auto-detects `/dev/videoN`, runs in its own process, zones use the 5th percentile | Opens index 0 (the webcam), blocks the vision thread, zones use the median (misses hands) |
| Red detection | Hue 0–6 / 172–179, saturation ≥150, blob ≥1% of frame | Hue 0–10 / 170–179, saturation ≥130, blob ≥0.4% (false reds on skin and tan) |
| Control loop | `--esp-log`, and `--log` for a per-frame CSV plus snapshots; shows the stop reason | |
| Ultrasonic ECHO pins | Pull-down (no sensors fitted) | Floating |

## Layout

```
navi_config.py    every tunable, cited to the report
navi_vision.py    webcam, HSV detection, vision and UI threads
navi_depth.py     Arducam ToF, left/center/right zones
navi_control.py   FSM, UART thread, --esp-log, --log
selftest.py       preflight check of all hardware
run_tests.sh      every virtual suite (129 Python tests, 410 firmware checks)

tools/            bench and diagnostic tools, run from this folder
  esp_link_test.py    8-step ESP32 test over USB
  tof_probe.py        live ToF zone readout
  tof_capture.py      guided ToF capture (Enter at each hand position)
  color_capture.py    guided colour capture (Enter holding each object)
  hsv_probe.py        HSV stats of test images
  make_test_images.py regenerate test_images/
  pi-wifi-rescue.ps1  add wifi to the Pi via its SD card (Windows)

firmware/         ESP-IDF project, USB/UART0 build
tests/            Python suites
test_images/      synthetic frames for the no-hardware tests
test_data/        everything recorded on the Pi (laptop only, not on the Pi)

docs/
  bench-log.md      how to run the bench tests, and the session results
  SYSTEMS.md        what is on the laptop, the Pi and the ESP32
  report-deltas.md  what the SD1 report needs changed
  COMMANDS.md       PCB/GPIO command reference (copied from NAVI)
  wiring.md         PCB pin map
```

## Quick start (on the Pi)

```bash
cd ~/willtest
./run_tests.sh                         # no hardware needed
python3 selftest.py                    # checks webcam, ToF, ESP32
python3 tools/esp_link_test.py         # ESP32 over USB, 12 checks
python3 navi_control.py --esp-log      # full system. m = AUTO, e = E-STOP, r = reset, q = quit
python3 navi_control.py --esp-log --log   # same, recording CSV and snapshots to output/
```

Output from the Pi goes to `~/willtest/output/`. Pull it back to `test_data/` and clear it
afterwards, as described in `docs/SYSTEMS.md`.
