// NAVI firmware logic: no ESP-IDF or hardware here, so it builds and tests on a host with g++.

#pragma once

#include <cstddef>
#include <cstdint>

namespace navi {

// Protocol constants

constexpr char START_BYTE = '<';
constexpr char STOP_BYTE = '>';
constexpr int PWM_MAX = 255;

// Longest legal payload is "-255,-255,1"; extra room so an overlong frame is caught as bad.
constexpr size_t MAX_PAYLOAD = 24;

// No valid packet inside this window drops the motors to zero.
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

// Byte-at-a-time parser. Never blocks or allocates; resyncs on the next '<'.
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

// Safety supervision

// Why the system is halted. Anything but NONE means motors at zero, relay open.
enum class HaltReason {
    NONE = 0,
    ESTOP_BUTTON,     // e stop interrupt fired
    SIGNAL_LOST,      // no valid packet inside 500 ms
    TILT,             // IMU reports an unsafe angle
    OBSTACLE,         // ultrasonic bumper
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

// Decide whether the chair may drive. Gates in order: E-stop, link, bumper, tilt.
// Defaults are closed: the chair starts halted.
SafetyDecision evaluate_safety(const SafetyInputs& in);

// Motor output shaping

int clamp_pwm(int value);

// Step toward target by at most max_step. Stops skip the ramp.
int ramp_toward(int current, int target, int max_step);

// Whole PWM steps allowed this pass at pwm_per_s, carrying the remainder. The first call returns 0.
// When it returns 0, hold the current output.
struct RampClock {
    int64_t last_us = -1;
    int64_t carry = 0;               // PWM x microseconds, always under one step
};

int ramp_steps(RampClock& clock, int64_t now_us, int pwm_per_s,
               int64_t max_gap_us);

// Ultrasonic

// Echo never came back. Kept apart from zero, which would read as an obstacle
// touching the sensor.
constexpr int32_t USS_NO_READING = -1;

// Echo width to mm (HC-SR04 / RCWL-1655): echo_us * 0.343 / 2. Out of range returns no reading.
constexpr int32_t USS_MIN_MM = 20;
constexpr int32_t USS_MAX_MM = 4000;
int32_t echo_us_to_mm(int64_t echo_us);

// Bumper with hysteresis: trips under stop_mm, releases after clear_ms at or past release_mm.
// No reading counts as clear.
struct BumperLatch {
    bool tripped = false;
    int64_t clear_since_us = -1;     // -1 while not clear
};

bool bumper_update(BumperLatch& latch, int32_t left_mm, int32_t right_mm,
                   int64_t now_us, int32_t stop_mm, int32_t release_mm,
                   int64_t clear_ms);

// ICM-20948 decode and tilt

struct ImuSample {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;   // g
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;   // degrees per second
};

// 12 bytes from ACCEL_XOUT_H: accel XYZ then gyro XYZ, big-endian int16.
ImuSample decode_icm_sample(const uint8_t* buf, float accel_lsb_per_g,
                            float gyro_lsb_per_dps);

// Gravity direction as a unit vector. Complementary filter on the vector, so any mounting works.
struct GravityFilter {
    bool ready = false;
    float x = 0.0f, y = 0.0f, z = 1.0f;
};

// accel_weight ~0.02. Samples more than accel_reject_g from 1 g use the gyro only.
void gravity_filter_update(GravityFilter& f, const ImuSample& s, float dt_s,
                           float accel_weight, float accel_reject_g);

// Up axis: 0..2 = +X/+Y/+Z, 3..5 = -X/-Y/-Z. Picked once at boot.
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

// True when tilted too far to drive.
bool tilt_exceeds(const Euler& e, float limit_deg);

}  // namespace navi
