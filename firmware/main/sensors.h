// IMU (ICM-20948, I2C) and ultrasonic polling on the ESP32.

#pragma once

#include <cstdint>

#include "navi_logic.h"

namespace navi {

struct SensorState {
    // IMU
    bool imu_ok = false;
    Euler euler;                  // roll and pitch from level, degrees
    float tilt_deg = 0.0f;        // total angle from level
    bool tilt_alarm = false;

    // Ultrasonic in mm, or USS_NO_READING
    bool uss_ok = false;          // trigger and echo found on the header
    int32_t uss_mm = USS_NO_READING;
    bool bumper = false;          // inside the stop distance
};

// Closer than this and the ESP32 opens the relays. Must match STOP_DISTANCE_MM in navi_config.py.
constexpr int32_t BUMPER_STOP_MM = 1540;

// Release only once it reads past this for BUMPER_CLEAR_MS straight.
// The gap above BUMPER_STOP_MM stops it chattering at the limit.
constexpr int32_t BUMPER_RELEASE_MM = 1740;
constexpr int64_t BUMPER_CLEAR_MS = 500;

// Past this roll or pitch the chair should not drive.
constexpr float TILT_LIMIT_DEG = 15.0f;

// Start I2C and the ultrasonic. Returns false if the IMU didn't answer.
bool sensors_init();

// Read everything once. Called from the sensor task only.
void sensors_poll(SensorState& out);

// Start the background poll task. sensors_latest() copies out its last result.
void sensors_start_task();
void sensors_latest(SensorState& out);

// I2C scan: idle levels, every ACKing address, WHO_AM_I. At boot without the IMU, and on "i2cscan".
void sensors_i2c_scan();

// Ultrasonic header check ("ussdiag"). Runs inside the sensor task.
void sensors_request_uss_diag();

}  // namespace navi
