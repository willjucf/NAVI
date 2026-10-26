// NAVI sensor polling. ICM-20948 IMU over I2C and one ultrasonic (RCWL-1655
// or HC-SR04, same trigger and echo pulse).
//
// These run on the ESP32 instead of the Pi (report 3.7.9). They cost almost
// nothing to process and the bumper has to work even when the Pi is busy or
// silent.

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

// Closer than this and the ESP32 stops forward motion itself.
// This is the bumper from report 3.7.6, meant to work when the Pi is slow, the
// webcam is blocked or the ToF is wrong.
constexpr int32_t BUMPER_STOP_MM = 900;

// Release only once it reads past this for BUMPER_CLEAR_MS straight.
// The gap above BUMPER_STOP_MM stops it chattering at the limit.
constexpr int32_t BUMPER_RELEASE_MM = 1100;
constexpr int64_t BUMPER_CLEAR_MS = 500;

// Past this roll or pitch the chair should not drive. Reported to the Pi since
// the IMU does not command motors by itself (report 3.6.6).
constexpr float TILT_LIMIT_DEG = 15.0f;

// Start I2C and the ultrasonic. Returns false if the ICM-20948 did not answer.
// The ultrasonic still works and the IMU stays marked not ok instead of
// reporting zeros that look real.
bool sensors_init();

// Read everything once. Called from the sensor task only.
void sensors_poll(SensorState& out);

// Start the background poll task. sensors_latest() copies out its last result.
void sensors_start_task();
void sensors_latest(SensorState& out);

}  // namespace navi
