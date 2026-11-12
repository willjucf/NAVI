# NAVI-full

The full NAVI system with AUTO mode: the Pi steers toward green targets and the
ESP32 drives the motors with PWM. Needs motor PWM pins routed to the motor
controller, which the current MCU PCB doesn't have, so the chair runs the
assisted-driving build in the repo root instead.

| Button | Does |
|---|---|
| START | STOP → MANUAL |
| SWITCH | MANUAL ↔ AUTO |
| SHUTDOWN | tap twice: send a stop and exit |

```bash
python3 navi_app.py [--window] [--no-serial]
./install.sh
```

- `firmware/`: UART only, plus `beep`, `i2cscan`, `ussdiag` over the UART.
- `firmware_wireless/`: the same with WiFi, BLE and OTA updates.

Build and flash as in the root README. Shared Pi modules match the root copies.
