// NAVI ESP32 pin assignments.
//
// Taken from the Group 7 PCB layout rev 1.0. The schematic says WROOM 32D but
// the fitted module is a WROOM 32E. They are pin compatible so nothing changes.
// Keep in sync with ESP32_PINS in navi_config.py. See docs/wiring.md.

#pragma once

#include "driver/gpio.h"

// UART to the Pi 5
//
// Read before changing. The board wires Pi TXD to IO17 and Pi RXD to IO16.
// Most pinouts label IO16 as RX2 and IO17 as TX2, so following those labels
// would join TX to TX and nothing would arrive. The ESP32 can route UART to any
// pin, so we swap them here to match the board.
#define NAVI_UART_PORT      UART_NUM_2
#define NAVI_UART_TX_PIN    GPIO_NUM_16   // to Pi GPIO15 (RXD)
#define NAVI_UART_RX_PIN    GPIO_NUM_17   // from Pi GPIO14 (TXD)
#define NAVI_UART_BAUD      115200        // report 6.13

// Safety
//
// The stop button sits between this pin and ground. The pin idles high on its
// pull up and a press pulls it low. A jumper to ground does the same thing,
// which is how the e stop is tested before the real button is mounted.
#define NAVI_ESTOP_PIN      GPIO_NUM_12

// Motor cutoff relay through an NPN. Hardware half of the stop (report 3.9.5).
#define NAVI_RELAY_PIN      GPIO_NUM_14

// Indicators, both through an NPN.
#define NAVI_LED_PIN        GPIO_NUM_4
#define NAVI_BUZZER_PIN     GPIO_NUM_5

// Ultrasonics (report 3.7.6)
//
// One per side, never fired together because two HC SR04s hear each other.
//
// Wiring warning: on rev 1.0 each ECHO line only has a 510 ohm series resistor.
// That limits current but does not shift 5 V down to 3.3 V, so the pin's clamp
// diode is doing the work. The report calls for a divider or level shifter.
// Firmware cannot fix this. See docs/wiring.md.
#define NAVI_USS1_TRIG_PIN  GPIO_NUM_18   // left
#define NAVI_USS1_ECHO_PIN  GPIO_NUM_19
#define NAVI_USS2_TRIG_PIN  GPIO_NUM_27   // right
#define NAVI_USS2_ECHO_PIN  GPIO_NUM_26

// I2C for the BNO055 IMU (report 3.6)
#define NAVI_I2C_SDA_PIN    GPIO_NUM_21
#define NAVI_I2C_SCL_PIN    GPIO_NUM_22
#define NAVI_I2C_FREQ_HZ    400000

// Motor PWM
//
// Not on the rev 1.0 sheet because the motor controller interface is still
// being settled (report 6.12). Placeholder pins so the PWM can be scoped today.
#define NAVI_MOTOR_LEFT_PIN   GPIO_NUM_32
#define NAVI_MOTOR_RIGHT_PIN  GPIO_NUM_33
