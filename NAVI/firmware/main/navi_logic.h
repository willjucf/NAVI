// NAVI firmware logic core.
//
// No ESP IDF, FreeRTOS, registers or heap in here on purpose. It builds with a
// plain host g++ so the parser, watchdog math, ultrasonic conversion, IMU
// decode and PWM ramp can be unit tested on a laptop or the Pi with no ESP32.
//
// main.cpp, safety.cpp and sensors.cpp are thin hardware wrappers around this.
// Anything that does not touch a peripheral belongs here.

#pragma once

#include <cstddef>
#include <cstdint>

namespace navi {

// Protocol constants (report 6.13)

constexpr char START_BYTE = '<';
constexpr char STOP_BYTE = '>';
constexpr int PWM_MAX = 255;

// Longest payload accepted between the framing bytes. The widest legal packet
// is "-255,-255,1" (11 chars). The extra room lets a bad frame be caught as
// bad instead of truncating a good one.
constexpr size_t MAX_PAYLOAD = 24;

// No valid packet inside this window drops the motors to zero (report 6.13, 6.16).
constexpr int64_t WATCHDOG_TIMEOUT_MS = 500;

// Drive command

struct DriveCommand {
    int left = 0;
    int right = 0;
    bool safety_flag = false;
};

// Why a frame was rejected. Anything but NONE means it was dropped and the
// previous command stands until the watchdog expires.
enum class ParseError {
    NONE = 0,
    PACKET_CORRUPTION_ERROR,  // payload was not three parseable integers
    PAYLOAD_TOO_LONG,         // no stop byte within a sane distance
};

// Byte at a time packet parser
//
// Never blocks or allocates, so it can run straight off a non blocking UART
// read. Bytes outside a frame are thrown away, so noise or a half packet from a
// reset just resyncs on the next start byte instead of corrupting good data.
class PacketParser {
public:
    // True when this byte finished a valid frame and out holds the command.
    // On false, last_error() tells "still collecting" apart from "bad frame".
    bool feed(char byte, DriveCommand& out);

    void reset();

    ParseError last_error() const { return error_; }
    uint32_t good_frames() const { return good_frames_; }
    uint32_t bad_frames() const { return bad_frames_; }

private:
    bool in_frame_ = false;
    size_t len_ = 0;
    char buf_[MAX_PAYLOAD + 1] = {0};
    ParseError error_ = ParseError::NONE;
    uint32_t good_frames_ = 0;
    uint32_t bad_frames_ = 0;
};

// Parse a bare payload like "150,-150,1". Separate so it can be tested directly.
bool parse_payload(const char* payload, size_t len, DriveCommand& out);

// Build the wire format. Shared by tests and status echoes so both ends use one
// definition. Returns characters written, not counting the terminator.
size_t format_packet(const DriveCommand& cmd, char* buf, size_t buf_len);

// Timing

// True when now_us is at least timeout_ms past last_us. The clock is passed in
// instead of read inside so the 500 ms boundary can be tested exactly.
bool deadline_expired(int64_t now_us, int64_t last_us, int64_t timeout_ms);

// Safety supervision (report 6.4, 6.6, 6.13, 6.15)

// Why the system is halted. Anything but NONE means motors at zero, relay open.
enum class HaltReason {
    NONE = 0,
    ESTOP_BUTTON,     // e stop interrupt fired
    SIGNAL_LOST,      // no valid packet inside 500 ms
    TILT,             // IMU reports an unsafe angle
    OBSTACLE,         // ultrasonic bumper, report 3.7.6
    NO_LINK,          // Pi never heard from since boot
};

const char* halt_reason_name(HaltReason reason);

// Everything the halt decision depends on.
struct SafetyInputs {
    int64_t now_us = 0;
    int64_t last_packet_us = 0;
    bool link_established = false;   // at least one valid packet decoded
    bool estop_latched = false;      // set by the e stop interrupt
    bool bumper = false;             // ultrasonic inside the stop distance
    bool imu_ok = false;
    bool tilt_alarm = false;
};

struct SafetyDecision {
    bool halted = true;
    HaltReason reason = HaltReason::NO_LINK;
    bool arm_relay = false;          // close the motor cutoff relay
};

// Decide whether the chair may drive.
//
// Kept pure so it can be tested. The two worst bugs so far were both here and
// both invisible on a bench: the relay never closed on the normal path, and
// the halt check read a torn timestamp.
//
// Gate order follows report 6.6: button, then link, then sensors. Defaults are
// closed, so the chair starts halted and has to earn the right to drive.
SafetyDecision evaluate_safety(const SafetyInputs& in);

// Motor output shaping (report 6.17)

int clamp_pwm(int value);

// Step current toward target by at most max_step. A sudden full scale command
// would jerk a seated rider, so speed is ramped. Stops skip the ramp and are
// applied by the caller immediately, since a stop that eases in is not a stop.
int ramp_toward(int current, int target, int max_step);

// Ultrasonic (report 3.7.6)

// Echo never came back. Kept apart from zero, which would read as an obstacle
// touching the sensor.
constexpr int32_t USS_NO_READING = -1;

// Echo width to mm (HC-SR04 and RCWL-1655 use the same pulse). Sound is about 343 m/s and the pulse is a round
// trip, so mm = echo_us * 0.343 / 2. Out of range readings return no reading
// because the HC SR04 does not fail cleanly out there.
constexpr int32_t USS_MIN_MM = 20;
constexpr int32_t USS_MAX_MM = 4000;
int32_t echo_us_to_mm(int64_t echo_us);

// Bumper with hysteresis. Trips at once on any reading under stop_mm, but only
// releases after every reading has been at or past release_mm (or no reading)
// for clear_ms straight. Without this a hand near the limit, a dropped echo or
// a ghost reading toggles the motors several times a second.
//
// No reading counts as clear, because open space reads that way. The hold
// time is what stops a single dropout from releasing it.
struct BumperLatch {
    bool tripped = false;
    int64_t clear_since_us = -1;     // -1 while not clear
};

bool bumper_update(BumperLatch& latch, int32_t left_mm, int32_t right_mm,
                   int64_t now_us, int32_t stop_mm, int32_t release_mm,
                   int64_t clear_ms);

// ICM-20948 decode and tilt (report 3.6)
//
// The ICM has no onboard fusion like the BNO055 did, so tilt is worked out
// here from the raw accelerometer and gyro.

struct ImuSample {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;   // g
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;   // degrees per second
};

// buf is 12 bytes from ACCEL_XOUT_H (0x2D, bank 0): accel XYZ then gyro XYZ,
// each a signed 16 bit big endian value. The scales depend on the full scale
// range set at init.
ImuSample decode_icm_sample(const uint8_t* buf, float accel_lsb_per_g,
                            float gyro_lsb_per_dps);

// Gravity direction in the sensor frame, as a unit vector.
//
// A complementary filter on the vector itself instead of on roll and pitch, so
// it works whichever way the board is mounted and has no sign conventions to
// get wrong. The gyro carries it through bumps and braking; the accelerometer
// pulls it back slowly so gyro drift cannot build up.
struct GravityFilter {
    bool ready = false;
    float x = 0.0f, y = 0.0f, z = 1.0f;
};

// accel_weight is the share of the accelerometer per update, about 0.02.
// Samples whose magnitude is more than accel_reject_g away from 1 g are being
// shaken or braked, so only the gyro is used for them.
void gravity_filter_update(GravityFilter& f, const ImuSample& s, float dt_s,
                           float accel_weight, float accel_reject_g);

// Which sensor axis points up: 0..2 = +X, +Y, +Z, 3..5 = -X, -Y, -Z. Picks the
// axis closest to the gravity vector, so any mounting that is square to the
// chair works. Chosen once at boot.
int up_axis(const GravityFilter& f);

// Angle between the axis and the gravity vector, degrees. At boot this is how
// far the board is from square; while driving it is the total tilt.
float angle_from_axis(const GravityFilter& f, int axis);

struct Euler {
    float heading = 0.0f;   // degrees, unused without a magnetometer
    float roll = 0.0f;
    float pitch = 0.0f;
};

// Roll and pitch relative to the up axis. Signs follow the sensor's axes, so
// only the size is meaningful until the mounting is known.
Euler tilt_angles(const GravityFilter& f, int axis);

// True when tilted too far to drive. The IMU does not command motors (report
// 3.6.6), so this goes to the Pi as a flag instead of acting locally.
bool tilt_exceeds(const Euler& e, float limit_deg);

}  // namespace navi
