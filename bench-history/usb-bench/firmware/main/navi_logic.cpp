// NAVI firmware logic core. See navi_logic.h for why this file has no ESP IDF
// dependencies.

#include "navi_logic.h"

#include <cstdio>
#include <cstdlib>

namespace navi {

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

namespace {

// Parse a signed decimal integer from [p, end). Returns false on an empty
// field, a stray character, or anything that would not fit in an int.
//
// Written by hand rather than with strtol so the parser never depends on a
// null terminator sitting in the right place, and so a field like "12x" is
// rejected instead of quietly read as 12.
bool parse_int(const char* p, const char* end, int& out) {
    if (p >= end) {
        return false;
    }
    bool negative = false;
    if (*p == '-' || *p == '+') {
        negative = (*p == '-');
        ++p;
        if (p >= end) {
            return false;
        }
    }
    long value = 0;
    for (; p < end; ++p) {
        if (*p < '0' || *p > '9') {
            return false;
        }
        value = value * 10 + (*p - '0');
        if (value > 100000L) {       // far past anything legal, stop early
            return false;
        }
    }
    out = static_cast<int>(negative ? -value : value);
    return true;
}

}  // namespace

bool parse_payload(const char* payload, size_t len, DriveCommand& out) {
    // Expect exactly three comma separated fields.
    const char* end = payload + len;
    const char* first = nullptr;
    const char* second = nullptr;

    for (const char* p = payload; p < end; ++p) {
        if (*p == ',') {
            if (first == nullptr) {
                first = p;
            } else if (second == nullptr) {
                second = p;
            } else {
                return false;        // a fourth field, not our format
            }
        }
    }
    if (first == nullptr || second == nullptr) {
        return false;
    }

    int left = 0;
    int right = 0;
    int flag = 0;
    if (!parse_int(payload, first, left)) {
        return false;
    }
    if (!parse_int(first + 1, second, right)) {
        return false;
    }
    if (!parse_int(second + 1, end, flag)) {
        return false;
    }
    if (flag != 0 && flag != 1) {
        return false;                // the safety flag is a boolean, not a number
    }

    // Clamp rather than reject an out of range PWM. A packet whose framing and
    // structure are intact is a packet the Pi meant to send, and refusing it
    // outright would mean holding the previous command until the watchdog
    // fires, which is worse than driving the value the range allows.
    out.left = clamp_pwm(left);
    out.right = clamp_pwm(right);
    out.safety_flag = (flag != 0);
    return true;
}

void PacketParser::reset() {
    in_frame_ = false;
    len_ = 0;
    error_ = ParseError::NONE;
}

bool PacketParser::feed(char byte, DriveCommand& out) {
    error_ = ParseError::NONE;

    if (byte == START_BYTE) {
        // Always restart here, even mid frame. A start byte inside a payload
        // means the previous frame was truncated by noise or a reset, and the
        // newer data is the one worth keeping.
        in_frame_ = true;
        len_ = 0;
        return false;
    }

    if (!in_frame_) {
        return false;                // pre frame garbage, discard silently
    }

    if (byte == STOP_BYTE) {
        in_frame_ = false;
        buf_[len_] = '\0';
        if (parse_payload(buf_, len_, out)) {
            ++good_frames_;
            return true;
        }
        ++bad_frames_;
        error_ = ParseError::PACKET_CORRUPTION_ERROR;
        return false;
    }

    if (len_ >= MAX_PAYLOAD) {
        // No stop byte within a plausible distance. Abandon the frame rather
        // than running off the end of the buffer, and wait for the next start.
        in_frame_ = false;
        len_ = 0;
        ++bad_frames_;
        error_ = ParseError::PAYLOAD_TOO_LONG;
        return false;
    }

    buf_[len_++] = byte;
    return false;
}

size_t format_packet(const DriveCommand& cmd, char* buf, size_t buf_len) {
    int written = snprintf(buf, buf_len, "<%d,%d,%d>",
                           clamp_pwm(cmd.left), clamp_pwm(cmd.right),
                           cmd.safety_flag ? 1 : 0);
    if (written < 0 || static_cast<size_t>(written) >= buf_len) {
        if (buf_len > 0) {
            buf[0] = '\0';
        }
        return 0;
    }
    return static_cast<size_t>(written);
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

bool deadline_expired(int64_t now_us, int64_t last_us, int64_t timeout_ms) {
    return (now_us - last_us) >= (timeout_ms * 1000);
}

// ---------------------------------------------------------------------------
// Safety supervision
// ---------------------------------------------------------------------------

const char* halt_reason_name(HaltReason reason) {
    switch (reason) {
        case HaltReason::ESTOP_BUTTON: return "ESTOP_BUTTON";
        case HaltReason::SIGNAL_LOST:  return "SIGNAL_LOST";
        case HaltReason::TILT:         return "TILT";
        case HaltReason::OBSTACLE:     return "OBSTACLE";
        case HaltReason::NO_LINK:      return "NO_LINK";
        default:                       return "NONE";
    }
}

SafetyDecision evaluate_safety(const SafetyInputs& in) {
    SafetyDecision out;
    out.halted = true;
    out.arm_relay = false;

    // Gate one, the physical stop. Report 6.6 puts this ahead of everything
    // else, and report 6.15 makes it inescapable without a deliberate reset.
    if (in.estop_latched) {
        out.reason = HaltReason::ESTOP_BUTTON;
        return out;
    }

    // Gate two, the link. Never heard from is a distinct state from went quiet,
    // and it is reported separately because the two mean different things on a
    // bench: one is a wiring or configuration problem, the other is a fault.
    if (!in.link_established) {
        out.reason = HaltReason::NO_LINK;
        return out;
    }
    if (deadline_expired(in.now_us, in.last_packet_us, WATCHDOG_TIMEOUT_MS)) {
        out.reason = HaltReason::SIGNAL_LOST;
        return out;
    }

    // Gate three, the local sensors. These run on this board specifically so
    // they still work when the Pi is delayed or the cameras are wrong.
    if (in.bumper) {
        out.reason = HaltReason::OBSTACLE;
        return out;
    }
    // A tilt reading is only trusted when the IMU is actually answering. A
    // failed read must not be interpreted as a level chair, and equally must
    // not fake a tilt, so the reading is simply not consulted.
    if (in.imu_ok && in.tilt_alarm) {
        out.reason = HaltReason::TILT;
        return out;
    }

    out.halted = false;
    out.reason = HaltReason::NONE;
    out.arm_relay = true;
    return out;
}

// ---------------------------------------------------------------------------
// Motor output shaping
// ---------------------------------------------------------------------------

int clamp_pwm(int value) {
    if (value > PWM_MAX) {
        return PWM_MAX;
    }
    if (value < -PWM_MAX) {
        return -PWM_MAX;
    }
    return value;
}

int ramp_toward(int current, int target, int max_step) {
    if (max_step <= 0) {
        return target;               // ramping disabled, jump straight there
    }
    const int delta = target - current;
    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

// ---------------------------------------------------------------------------
// Ultrasonic
// ---------------------------------------------------------------------------

int32_t echo_us_to_mm(int64_t echo_us) {
    if (echo_us <= 0) {
        return USS_NO_READING;
    }
    // 343 m/s is 0.343 mm per microsecond, halved for the round trip.
    const int32_t mm = static_cast<int32_t>((echo_us * 343) / 2000);
    if (mm < USS_MIN_MM || mm > USS_MAX_MM) {
        return USS_NO_READING;
    }
    return mm;
}

// ---------------------------------------------------------------------------
// BNO055 decode
// ---------------------------------------------------------------------------

namespace {

int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}

}  // namespace

Euler decode_euler(const uint8_t* buf) {
    Euler e;
    e.heading = le16(buf + 0) / 16.0f;
    e.roll = le16(buf + 2) / 16.0f;
    e.pitch = le16(buf + 4) / 16.0f;
    return e;
}

CalibStatus decode_calib(uint8_t reg) {
    CalibStatus c;
    c.system = (reg >> 6) & 0x03;
    c.gyro = (reg >> 4) & 0x03;
    c.accel = (reg >> 2) & 0x03;
    c.mag = reg & 0x03;
    return c;
}

bool tilt_exceeds(const Euler& e, float limit_deg) {
    const float roll = e.roll < 0 ? -e.roll : e.roll;
    const float pitch = e.pitch < 0 ? -e.pitch : e.pitch;
    return roll > limit_deg || pitch > limit_deg;
}

}  // namespace navi
