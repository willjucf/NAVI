// NAVI sensor polling. BNO055 IMU over I2C and two HC SR04 ultrasonics.
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
    Euler euler;
    CalibStatus calib;
    bool tilt_alarm = false;

    // Ultrasonics in mm, or USS_NO_READING
    int32_t left_mm = USS_NO_READING;
    int32_t right_mm = USS_NO_READING;
    bool bumper = false;          // either side inside the stop distance
};

// Closer than this on either side and the ESP32 stops forward motion itself.
// This is the bumper from report 3.7.6, meant to work when the Pi is slow, the
// webcam is blocked or the ToF is wrong.
constexpr int32_t BUMPER_STOP_MM = 300;

// Release only once both sides read past this for BUMPER_CLEAR_MS straight.
// The gap above BUMPER_STOP_MM stops it chattering at the limit.
constexpr int32_t BUMPER_RELEASE_MM = 400;
constexpr int64_t BUMPER_CLEAR_MS = 500;

// Past this roll or pitch the chair should not drive. Reported to the Pi since
// the IMU does not command motors by itself (report 3.6.6).
constexpr float TILT_LIMIT_DEG = 15.0f;

// Start I2C and the ultrasonic pins. Returns false if the BNO055 did not
// answer. The ultrasonics still work and the IMU stays marked not ok instead
// of reporting zeros that look real.
bool sensors_init();

// Read everything once. Safe from a task at any rate. The ultrasonics take
// turns so they never fire together.
void sensors_poll(SensorState& out);

// Start the background poll task. sensors_latest() copies out its last result.
void sensors_start_task();
void sensors_latest(SensorState& out);

}  // namespace navi
