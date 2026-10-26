// NAVI firmware logic core.
//
// Everything in this header is deliberately free of ESP IDF includes, FreeRTOS
// calls, hardware registers and dynamic allocation. That is the whole point: it
// compiles with a plain host g++ so the packet parser, the watchdog deadline
// arithmetic, the ultrasonic conversion, the IMU register decode and the PWM
// ramp can all be unit tested on a laptop or on the Pi, with no ESP32 attached
// and no flashing cycle.
//
// main.cpp, safety.cpp and sensors.cpp are thin hardware wrappers around these
// functions. If a behaviour can be expressed without touching a peripheral, it
// belongs here rather than there.

#pragma once

#include <cstddef>
#include <cstdint>

namespace navi {

// ---------------------------------------------------------------------------
// Protocol constants (report 6.13)
// ---------------------------------------------------------------------------

constexpr char START_BYTE = '<';
constexpr char STOP_BYTE = '>';
constexpr int PWM_MAX = 255;

// Longest payload we will ever accept between the framing bytes. The widest
// legal packet is "-255,-255,1" at eleven characters, so this leaves room for
// a malformed frame to be recognised as malformed rather than silently
// truncating a valid one.
constexpr size_t MAX_PAYLOAD = 24;

// Report 6.13 and 6.16. If no valid packet arrives inside this window the
// motors are dropped to zero and the chair halts.
constexpr int64_t WATCHDOG_TIMEOUT_MS = 500;

// ---------------------------------------------------------------------------
// Drive command
// ---------------------------------------------------------------------------

struct DriveCommand {
    int left = 0;
    int right = 0;
    bool safety_flag = false;
};

// Why a frame was rejected. Anything other than NONE means the frame was
// discarded and the previous command still stands, until the watchdog expires.
enum class ParseError {
    NONE = 0,
    PACKET_CORRUPTION_ERROR,  // payload was not three parseable integers
    PAYLOAD_TOO_LONG,         // never saw a stop byte in a sane distance
};

// ---------------------------------------------------------------------------
// Byte at a time packet parser
// ---------------------------------------------------------------------------

// Feed it one received byte at a time. It never blocks and never allocates, so
// it can be driven straight from a non blocking UART read loop.
//
// Any byte outside a frame is discarded, which means line noise, a half packet
// left over from a reset, or the tail of a frame whose start byte was missed
// all resynchronise on the next start byte rather than corrupting a good
// packet behind them.
class PacketParser {
public:
    // Returns true when this byte completed a valid frame, in which case out
    // holds the decoded command. Returns false otherwise; check last_error()
    // to distinguish "still collecting" from "that frame was bad".
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

// Parse a bare payload such as "150,-150,1" into a command.
// Exposed separately so it can be tested directly.
bool parse_payload(const char* payload, size_t len, DriveCommand& out);

// Build the wire format. Used by loopback tests and by any status the firmware
// echoes back, so both ends of the link share one definition of the format.
// Returns the number of characters written, not counting the terminator.
size_t format_packet(const DriveCommand& cmd, char* buf, size_t buf_len);

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

// True when now_us is at least timeout_ms past last_us.
//
// Takes the clock as an argument rather than reading esp_timer_get_time()
// internally, which is what lets the 500 ms boundary be tested exactly instead
// of approximately.
bool deadline_expired(int64_t now_us, int64_t last_us, int64_t timeout_ms);

// ---------------------------------------------------------------------------
// Safety supervision (report 6.4, 6.6, 6.13, 6.15)
// ---------------------------------------------------------------------------

// Why the system is halted. Anything other than NONE means the motors are at
// zero and the cutoff relay is open.
enum class HaltReason {
    NONE = 0,
    ESTOP_BUTTON,     // the e stop interrupt fired
    SIGNAL_LOST,      // no valid packet inside the 500 ms window
    TILT,             // the IMU reported an unsafe angle
    OBSTACLE,         // an ultrasonic bumper reading, report 3.7.6
    NO_LINK,          // the Pi has never been heard from since boot
};

const char* halt_reason_name(HaltReason reason);

// Everything the halt decision depends on, gathered in one place.
struct SafetyInputs {
    int64_t now_us = 0;
    int64_t last_packet_us = 0;
    bool link_established = false;   // a valid packet has been decoded at least once
    bool estop_latched = false;      // hardware latch set by the e stop interrupt
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
// This is a pure function on purpose. The two worst bugs this firmware has had
// were both in exactly this decision and both were invisible on a bench: the
// relay was never closed on the normal path, so the chair looked alive and
// could not move, and the halt state was evaluated against a torn timestamp.
// Neither is catchable by a compiler, and catching them on hardware means
// noticing an absence. Keeping the rule here means it is tested instead.
//
// The ordering is the safety gate ordering of report 6.6. The button outranks
// everything, then the link, then the sensors.
//
// Defaults are deliberately closed: the chair starts halted and has to earn the
// right to drive, rather than starting live and having to be stopped.
SafetyDecision evaluate_safety(const SafetyInputs& in);

// ---------------------------------------------------------------------------
// Motor output shaping (report 6.17)
// ---------------------------------------------------------------------------

int clamp_pwm(int value);

// Move current one step toward target, by at most max_step.
//
// Report 6.17: translating a sudden full scale command straight into full duty
// cycle would jerk the chair hard enough to matter for a seated rider, so the
// PWM is ramped instead. A stop is the deliberate exception and is applied
// immediately by the caller rather than ramped, because a safety stop that
// eases in is not a safety stop.
int ramp_toward(int current, int target, int max_step);

// ---------------------------------------------------------------------------
// Ultrasonic (report 3.7.6)
// ---------------------------------------------------------------------------

// Returned when the echo never came back inside the timeout. Distinct from a
// distance of zero, which would read as an obstacle pressed against the sensor.
constexpr int32_t USS_NO_READING = -1;

// Convert an HC SR04 echo pulse width to millimetres.
//
// Sound covers roughly 343 m/s at room temperature, and the pulse measures the
// round trip, so distance in mm is echo_us * 0.343 / 2. Readings outside the
// sensor usable range are reported as no reading rather than as a confident
// number, because an out of range HC SR04 does not fail cleanly.
constexpr int32_t USS_MIN_MM = 20;
constexpr int32_t USS_MAX_MM = 4000;
int32_t echo_us_to_mm(int64_t echo_us);

// ---------------------------------------------------------------------------
// BNO055 decode (report 3.6)
// ---------------------------------------------------------------------------

struct Euler {
    float heading = 0.0f;   // degrees
    float roll = 0.0f;
    float pitch = 0.0f;
};

// The BNO055 returns Euler angles as three signed 16 bit little endian values,
// each 1/16 of a degree. buf must point at six bytes starting from register
// 0x1A. Decoding it here rather than inside the I2C driver means it can be
// checked against known register contents with no hardware present.
Euler decode_euler(const uint8_t* buf);

struct CalibStatus {
    uint8_t system = 0;      // each field is 0 to 3, 3 meaning fully calibrated
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
};

CalibStatus decode_calib(uint8_t reg);

// True when the chair is tilted far enough that it should not be driving.
// The IMU does not command the motors (report 3.6.6), so this is reported to
// the Pi as a flag rather than acted on locally.
bool tilt_exceeds(const Euler& e, float limit_deg);

}  // namespace navi
