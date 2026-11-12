// ESP32 pins on the MCU PCB. Must match ESP32_PINS in navi_config.py.

#pragma once

#include "driver/gpio.h"

// UART to the Pi: Pi USB -> CP2102 (Vision PCB) -> MCU P3 -> UART0.
#define NAVI_UART_PORT      UART_NUM_0
#define NAVI_UART_TX_PIN    GPIO_NUM_1    // TXD0, P3 pin 4
#define NAVI_UART_RX_PIN    GPIO_NUM_3    // RXD0, P3 pin 5
#define NAVI_UART_BAUD      115200

// E-stop: normally closed to ground. Pressed, cut or unplugged all read high = stop.
#define NAVI_ESTOP_PIN      GPIO_NUM_25
#define NAVI_ESTOP_PRESSED  1             // pin level while stopped

// Motor cutoff relay through an NPN (P2 pin 4). Hardware half of the stop
//.
#define NAVI_RELAY_PIN      GPIO_NUM_14

// Indicators, both through an NPN (P2 pins 3 and 2).
#define NAVI_LED_PIN        GPIO_NUM_4
#define NAVI_BUZZER_PIN     GPIO_NUM_5

// Ultrasonic on U1 (IO26/IO27). Trigger and echo are detected at boot.
// No divider on echo: run the sensor from 3.3 V or add 1k/2k on the cable.
#define NAVI_USS_PIN_A      GPIO_NUM_26   // U1 pin 2
#define NAVI_USS_PIN_B      GPIO_NUM_27   // U1 pin 3

// I2C for the onboard ICM-20948 IMU
#define NAVI_I2C_SDA_PIN    GPIO_NUM_21
#define NAVI_I2C_SCL_PIN    GPIO_NUM_22
#define NAVI_I2C_FREQ_HZ    400000

// Motor PWM: not routed on the PCB. Unused in assisted driving.
#define NAVI_MOTOR_LEFT_PIN   GPIO_NUM_32
#define NAVI_MOTOR_RIGHT_PIN  GPIO_NUM_33
