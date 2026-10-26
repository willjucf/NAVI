// NAVI ESP32 pin assignments.
//
// Taken from the final MCU PCB (EasyEDA export PCB_MCU-Final_2026-10-06).
// Keep in sync with ESP32_PINS in navi_config.py; selftest.py checks them.

#pragma once

#include "driver/gpio.h"

// UART to the Pi 5
//
// Pi USB -> CP2102 on the Vision PCB -> 3 wire harness -> MCU P3 -> UART0.
// The log console shares UART0, so log lines go up to the Pi on the same cable. The Pi only
// displays them and the packet parser ignores anything outside <...>.
#define NAVI_UART_PORT      UART_NUM_0
#define NAVI_UART_TX_PIN    GPIO_NUM_1    // TXD0, P3 pin 4
#define NAVI_UART_RX_PIN    GPIO_NUM_3    // RXD0, P3 pin 5
#define NAVI_UART_BAUD      115200        // report 6.13

// Safety
//
// The stop button sits between this pin and ground (aux header P2 pin 1). The
// pin idles high on its pull up and a press pulls it low. A jumper to ground
// does the same thing. Moved off IO12, which is a strapping pin.
#define NAVI_ESTOP_PIN      GPIO_NUM_25

// Motor cutoff relay through an NPN (P2 pin 4). Hardware half of the stop
// (report 3.9.5).
#define NAVI_RELAY_PIN      GPIO_NUM_14

// Indicators, both through an NPN (P2 pins 3 and 2).
#define NAVI_LED_PIN        GPIO_NUM_4
#define NAVI_BUZZER_PIN     GPIO_NUM_5

// Ultrasonic (report 3.7.6)
//
// One sensor on JST U1: 1 = 5 V, 2 = IO26, 3 = IO27, 4 = GND. Which of the two
// is trigger and which is echo depends on the cable and the sensor's pin order,
// so sensors.cpp works it out at boot. These are just the two header pins.
//
// Wiring warning: there is no resistor on either line. A sensor powered from
// the 5 V pin drives echo at 5 V into a 3.3 V pin. Use a divider on the cable
// (1k series, 2k to GND), or run an RCWL-1655 from 3.3 V instead.
#define NAVI_USS_PIN_A      GPIO_NUM_26   // U1 pin 2
#define NAVI_USS_PIN_B      GPIO_NUM_27   // U1 pin 3

// I2C for the onboard ICM-20948 IMU (report 3.6)
#define NAVI_I2C_SDA_PIN    GPIO_NUM_21
#define NAVI_I2C_SCL_PIN    GPIO_NUM_22
#define NAVI_I2C_FREQ_HZ    400000

// Motor PWM
//
// Not routed on the final PCB. The motor controller interface is still being
// settled (report 6.12). Placeholder pins so the PWM can be scoped today.
#define NAVI_MOTOR_LEFT_PIN   GPIO_NUM_32
#define NAVI_MOTOR_RIGHT_PIN  GPIO_NUM_33
