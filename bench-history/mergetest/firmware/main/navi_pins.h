// NAVI ESP32 pin assignments.
//
// Read directly off Group 7 NAVI PCB Layout rev 1.0 (EasyEDA, 2026 08 28).
// The schematic sheet prints ESP32 WROOM 32D. The module actually fitted is a
// WROOM 32E, which is pin compatible with the same footprint, so nothing here
// changes between the two.
//
// Keep this in sync with ESP32_PINS in navi_config.py on the Pi side.
// See docs/wiring.md for the reasoning behind the two surprising entries.

#pragma once

#include "driver/gpio.h"

// ---------------------------------------------------------------------------
// UART to the Raspberry Pi 5
// ---------------------------------------------------------------------------
// WILLTEST: bench build for a plain ESP32 dev board plugged into the Pi by USB.
// The dev board's USB bridge is wired to UART0 (GPIO1 TX, GPIO3 RX), so the
// packets arrive there instead of on IO16/IO17. The console also lives on
// UART0, so the ESP32's log lines travel back up the same cable, which is
// harmless (the Pi only reads them for display) and means no second adapter
// is needed to watch the firmware. The real PCB build is in ../../firmware.
#define NAVI_UART_PORT      UART_NUM_0
#define NAVI_UART_TX_PIN    GPIO_NUM_1    // dev board USB bridge RX
#define NAVI_UART_RX_PIN    GPIO_NUM_3    // dev board USB bridge TX
#define NAVI_UART_BAUD      115200        // report 6.13

// Original PCB wiring, kept for reference:
//
// READ THIS BEFORE CHANGING EITHER NUMBER.
//
// The board wires Pi GPIO14 (TXD) to IO17 and Pi GPIO15 (RXD) to IO16. Almost
// every ESP32 pinout diagram labels IO16 as RX2 and IO17 as TX2, and wiring it
// that way here would connect TX to TX and RX to RX, so nothing would ever
// arrive and both sides would look correct in isolation. The ESP32 UART pin
// matrix is fully remappable, so we assign the opposite of the common labels
// and the board is right.
//   UART_NUM_2, TX GPIO_NUM_16 (to Pi GPIO15), RX GPIO_NUM_17 (from Pi GPIO14)

// ---------------------------------------------------------------------------
// Safety
// ---------------------------------------------------------------------------
// The stop button normally open contact sits between this pin and ground, so
// the pin idles high on its internal pull up and a press pulls it low. That is
// also exactly what a breadboard jumper to ground does, which is how the
// virtual e stop is tested before the real button is mounted.
#define NAVI_ESTOP_PIN      GPIO_NUM_12

// Motor cutoff relay, driven through an NPN with a 1K base resistor.
// This is the hardware half of the stop described in report 3.9.5.
#define NAVI_RELAY_PIN      GPIO_NUM_14

// Indicators, both driven through an NPN with a 1K base resistor.
#define NAVI_LED_PIN        GPIO_NUM_4
#define NAVI_BUZZER_PIN     GPIO_NUM_5

// ---------------------------------------------------------------------------
// Ultrasonic sensors (report 3.7.6)
// ---------------------------------------------------------------------------
// One sensor per side. They are never triggered at the same moment, because
// two HC SR04 units firing together hear each other and report nonsense.
//
// WIRING WARNING: on rev 1.0 each ECHO line reaches the ESP32 through a single
// 510 ohm series resistor. That is a current limit, not a level shift, so a 5 V
// echo is presented to a 3.3 V pin and only the internal clamp diode is holding
// it back. Report 3.7.6 asks for a divider and 7.3 asks for a level shifter.
// See docs/wiring.md. Firmware cannot fix this.
#define NAVI_USS1_TRIG_PIN  GPIO_NUM_18   // left
#define NAVI_USS1_ECHO_PIN  GPIO_NUM_19
#define NAVI_USS2_TRIG_PIN  GPIO_NUM_27   // right
#define NAVI_USS2_ECHO_PIN  GPIO_NUM_26

// ---------------------------------------------------------------------------
// I2C, BNO055 IMU (report 3.6)
// ---------------------------------------------------------------------------
#define NAVI_I2C_SDA_PIN    GPIO_NUM_21
#define NAVI_I2C_SCL_PIN    GPIO_NUM_22
#define NAVI_I2C_FREQ_HZ    400000

// ---------------------------------------------------------------------------
// Motor PWM outputs
// ---------------------------------------------------------------------------
// Not present on the rev 1.0 sheet, since the existing motor controller is
// being reused and the interface to it is still being settled (report 6.12).
// These are placeholders on otherwise unused pins so the PWM path is real and
// measurable on a scope today.
#define NAVI_MOTOR_LEFT_PIN   GPIO_NUM_32
#define NAVI_MOTOR_RIGHT_PIN  GPIO_NUM_33
