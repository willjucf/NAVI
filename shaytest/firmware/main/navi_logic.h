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

// HC SR04 echo width to mm. Sound is about 343 m/s and the pulse is a round
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

// BNO055 decode (report 3.6)

struct Euler {
    float heading = 0.0f;   // degrees
    float roll = 0.0f;
    float pitch = 0.0f;
};

// Euler angles are three signed 16 bit little endian values in 1/16 degree.
// buf is six bytes from register 0x1A. Decoding here instead of in the I2C
// driver lets it be checked against known bytes with no hardware.
Euler decode_euler(const uint8_t* buf);

struct CalibStatus {
    uint8_t system = 0;      // each 0 to 3, 3 is fully calibrated
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
};

CalibStatus decode_calib(uint8_t reg);

// True when tilted too far to drive. The IMU does not command motors (report
// 3.6.6), so this goes to the Pi as a flag instead of acting locally.
bool tilt_exceeds(const Euler& e, float limit_deg);

}  // namespace navi
