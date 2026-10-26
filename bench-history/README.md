# bench-history

Older code trees that were actually used for testing before the final PCBs. They're kept as the
record of how each result in `../NAVI/README.md` was produced. **Don't build new work on them.** The
current code is `../NAVI/`.

Each is a snapshot of the folder as it ran on the Pi. Test images, recordings and long docs were
left out (they're kept off GitHub).

| Folder | Dates | Hardware | What it tested, and how |
|---|---|---|---|
| `willtest/` | 2026-09-25 to 10-15 | Pi + webcam + ToF + ESP32 **dev kit on USB** (no ultrasonics, IMU or E-stop) | The first live tests from Will's room. `run_tests.sh` and `tests/` ran the unit tests on the Pi. `tools/color_capture.py` retuned red (09-25). `tools/esp_link_test.py` proved the Pi to ESP link. `tools/tof_probe.py` / `tof_capture.py` tuned the ToF zones. Its fixes were ported into `NAVI/`. |
| `shaytest/` | 2026-10-06 to 10-15 | Same dev-kit bench, plus an ultrasonic | Shay's branch: bumper hysteresis (trip/release with a 500 ms clear time), `firmware_tests/uss_test` (standalone ultrasonic with wiring diagnosis), and the ESP-IDF v6 build fix. Merged into `NAVI/` on 10-15. `navi_vision.py` has the `--fit` / `--fullscreen` LCD options it was tried with. |
| `usb-bench/` | 2026-10-15 | Dev kit on USB, ToF on CAM0, `/dev/ttyUSB0` hard-coded | Copy of `willtest` used to run the first interactive hardware tests (`tests-usb/`). ESP link 12/12, RCWL-1655 at 3.3 V read 41 to 43 cm. |
| `mergetest/` | 2026-10-15 to 10-22 | Dev kit on USB + webcam + ToF + 3.5" LCD | Full integration before the PCBs. `tests-interactive/test_final.py` ran auto and manual modes, the speed ramp, all sensors and the LCD. Red (S ≥ 140) and ToF zones (1 m / 2 m) were tuned here. Its Python code is what `NAVI/` now runs, with the PCB settings. |
| `tests-usb/` | 2026-10-15 | Dev kit on USB | First version of the four interactive tests (webcam, ToF, ultrasonic, ESP link), run from `usb-bench/`. Superseded by `NAVI/tests-interactive/`. |
| `tests-pcb/` | 2026-10-15 | Same, with port auto-detection | The same four tests adapted to find the CP2102 automatically, for the PCB link. Superseded by `NAVI/tests-interactive/`. |

To rerun one of these exactly, copy the folder to the Pi and follow its own README. Each one expects
the dev-kit wiring, not the PCBs.
