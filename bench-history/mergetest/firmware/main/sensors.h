// NAVI sensor polling. BNO055 IMU over I2C and two HC SR04 ultrasonics.
//
// Report 3.7.9 puts both of these on the ESP32 rather than the Pi, because
// their processing cost is near zero and because the ultrasonic bumper needs to
// work even when the Pi is busy or has stopped talking.

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

    // Ultrasonics, millimetres, or USS_NO_READING
    int32_t left_mm = USS_NO_READING;
    int32_t right_mm = USS_NO_READING;
    bool bumper = false;          // either side inside the stop distance
};

// Below this on either sensor, the ESP32 blocks further forward motion on its
// own. Report 3.7.6 calls this the bumper, and its whole purpose is to still
// work when the Pi is delayed, the webcam is blocked, or the ToF is lying.
constexpr int32_t BUMPER_STOP_MM = 900;

// Past this roll or pitch the chair should not be driving. Reported to the Pi
// rather than acted on locally, since report 3.6.6 is explicit that the IMU
// does not command the motors by itself.
constexpr float TILT_LIMIT_DEG = 15.0f;

// Bring up I2C and the ultrasonic pins. Returns false if the BNO055 did not
// answer, in which case the ultrasonics still work and the IMU fields stay
// marked not ok rather than reporting zeros that look like real data.
bool sensors_init();

// Read everything once. Safe to call from a task at any rate; the ultrasonics
// internally alternate so the two units never fire together.
void sensors_poll(SensorState& out);

// Start the background polling task. Results are published into a snapshot
// that sensors_latest() copies out.
void sensors_start_task();
void sensors_latest(SensorState& out);

}  // namespace navi
