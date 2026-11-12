// NAVI firmware logic core. See navi_logic.h for why there is no ESP IDF here.

#include "navi_logic.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace navi {

// Parsing

namespace {

// Parse a signed integer from [p, end). Rejects empty input, stray characters and overflow.
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
        if (value > 100000L) {       // way past anything legal, stop early
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
        return false;                // safety flag must be 0 or 1
    }

    // Clamp rather than reject a well-formed packet.
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
        // Always restart, even mid frame. A start byte inside a payload means
        // the last frame was cut short, and the newer data is what matters.
        in_frame_ = true;
        len_ = 0;
        return false;
    }

    if (!in_frame_) {
        return false;                // garbage before a frame, drop it
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
        // No stop byte in a sane distance. Drop the frame instead of overrunning
        // the buffer and wait for the next start.
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

// Timing

bool deadline_expired(int64_t now_us, int64_t last_us, int64_t timeout_ms) {
    return (now_us - last_us) >= (timeout_ms * 1000);
}

// Safety supervision

const char* halt_reason_name(HaltReason reason) {
    switch (reason) {
        case HaltReason::ESTOP_BUTTON: return "ESTOP_BUTTON";
        case HaltReason::SIGNAL_LOST:  return "SIGNAL_LOST";
        case HaltReason::TILT:         return "TILT";
        case HaltReason::OBSTACLE:     return "OBSTACLE";
        case HaltReason::NO_LINK:      return "NO_LINK";
        case HaltReason::PI_HAZARD:    return "PI_HAZARD";
        default:                       return "NONE";
    }
}

SafetyDecision evaluate_safety(const SafetyInputs& in) {
    SafetyDecision out;
    out.halted = true;
    out.arm_relay = false;

    // Gate 1: the E-stop. Cleared only by releasing the button.
    if (in.estop_latched) {
        out.reason = HaltReason::ESTOP_BUTTON;
        return out;
    }

    // Gate 2, the link. Never heard from and went quiet are reported apart
    // because one is a wiring or config problem and the other is a fault.
    if (!in.link_established) {
        out.reason = HaltReason::NO_LINK;
        return out;
    }
    if (deadline_expired(in.now_us, in.last_packet_us, WATCHDOG_TIMEOUT_MS)) {
        out.reason = HaltReason::SIGNAL_LOST;
        return out;
    }

    // Gate 3: the Pi's hazard flag (red, ToF obstacle or STOP on the screen).
    if (in.pi_hazard) {
        out.reason = HaltReason::PI_HAZARD;
        return out;
    }

    // Gate 4, local sensors. They run on this board so they still work when
    // the Pi is slow or the cameras are wrong.
    if (in.bumper) {
        out.reason = HaltReason::OBSTACLE;
        return out;
    }
    // Only trust tilt when the IMU is answering. A failed read should neither
    // look level nor fake a tilt, so it is just ignored.
    if (in.imu_ok && in.tilt_alarm) {
        out.reason = HaltReason::TILT;
        return out;
    }

    out.halted = false;
    out.reason = HaltReason::NONE;
    out.arm_relay = true;
    return out;
}

// Motor output shaping

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
        return target;               // ramp disabled, jump straight there
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

int ramp_steps(RampClock& clock, int64_t now_us, int pwm_per_s,
               int64_t max_gap_us) {
    if (clock.last_us < 0 || pwm_per_s <= 0) {
        clock.last_us = now_us;
        clock.carry = 0;
        return 0;
    }
    int64_t gap = now_us - clock.last_us;
    clock.last_us = now_us;
    if (gap < 0) {
        gap = 0;
    }
    if (gap > max_gap_us) {
        gap = max_gap_us;
    }
    clock.carry += gap * pwm_per_s;
    const int64_t steps = clock.carry / 1000000;
    clock.carry -= steps * 1000000;
    return static_cast<int>(steps);
}

// Ultrasonic

int32_t echo_us_to_mm(int64_t echo_us) {
    if (echo_us <= 0) {
        return USS_NO_READING;
    }
    // 343 m/s is 0.343 mm/us, halved for the round trip.
    const int32_t mm = static_cast<int32_t>((echo_us * 343) / 2000);
    if (mm < USS_MIN_MM || mm > USS_MAX_MM) {
        return USS_NO_READING;
    }
    return mm;
}

static bool uss_below(int32_t mm, int32_t limit_mm) {
    return mm != USS_NO_READING && mm < limit_mm;
}

bool bumper_update(BumperLatch& latch, int32_t left_mm, int32_t right_mm,
                   int64_t now_us, int32_t stop_mm, int32_t release_mm,
                   int64_t clear_ms) {
    // Tripping is never delayed.
    if (uss_below(left_mm, stop_mm) || uss_below(right_mm, stop_mm)) {
        latch.tripped = true;
        latch.clear_since_us = -1;
        return true;
    }
    if (!latch.tripped) {
        return false;
    }

    // Inside the band between stop and release keeps it held and restarts
    // the clear timer.
    if (uss_below(left_mm, release_mm) || uss_below(right_mm, release_mm)) {
        latch.clear_since_us = -1;
        return true;
    }
    if (latch.clear_since_us < 0) {
        latch.clear_since_us = now_us;
    }
    if (deadline_expired(now_us, latch.clear_since_us, clear_ms)) {
        latch.tripped = false;
        latch.clear_since_us = -1;
    }
    return latch.tripped;
}

// ICM-20948 decode and tilt

namespace {

int16_t be16(const uint8_t* p) {
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) |
                                static_cast<uint16_t>(p[1]));
}

constexpr float DEG_TO_RAD = 0.017453292f;
constexpr float RAD_TO_DEG = 57.29577951f;

float component(const GravityFilter& f, int axis) {
    const float v[3] = {f.x, f.y, f.z};
    return axis < 3 ? v[axis] : -v[axis - 3];
}

}  // namespace

ImuSample decode_icm_sample(const uint8_t* buf, float accel_lsb_per_g,
                            float gyro_lsb_per_dps) {
    ImuSample s;
    s.ax = be16(buf + 0) / accel_lsb_per_g;
    s.ay = be16(buf + 2) / accel_lsb_per_g;
    s.az = be16(buf + 4) / accel_lsb_per_g;
    s.gx = be16(buf + 6) / gyro_lsb_per_dps;
    s.gy = be16(buf + 8) / gyro_lsb_per_dps;
    s.gz = be16(buf + 10) / gyro_lsb_per_dps;
    return s;
}

void gravity_filter_update(GravityFilter& f, const ImuSample& s, float dt_s,
                           float accel_weight, float accel_reject_g) {
    const float mag = std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az);

    // Seed from the first sane accelerometer reading.
    if (!f.ready) {
        if (mag > 0.5f) {
            f.x = s.ax / mag;
            f.y = s.ay / mag;
            f.z = s.az / mag;
            f.ready = true;
        }
        return;
    }

    // Gyro step. A vector fixed in the world moves the opposite way to the
    // sensor's rotation, so v' = v - (w x v) dt, which is v + (v x w) dt.
    const float wx = s.gx * DEG_TO_RAD * dt_s;
    const float wy = s.gy * DEG_TO_RAD * dt_s;
    const float wz = s.gz * DEG_TO_RAD * dt_s;
    float x = f.x + (f.y * wz - f.z * wy);
    float y = f.y + (f.z * wx - f.x * wz);
    float z = f.z + (f.x * wy - f.y * wx);

    // Accelerometer correction, only when it is reading gravity alone.
    const float off = mag > 1.0f ? mag - 1.0f : 1.0f - mag;
    if (mag > 0.0f && off <= accel_reject_g) {
        x = (1.0f - accel_weight) * x + accel_weight * (s.ax / mag);
        y = (1.0f - accel_weight) * y + accel_weight * (s.ay / mag);
        z = (1.0f - accel_weight) * z + accel_weight * (s.az / mag);
    }

    const float n = std::sqrt(x * x + y * y + z * z);
    if (n > 0.0f) {
        f.x = x / n;
        f.y = y / n;
        f.z = z / n;
    }
}

int up_axis(const GravityFilter& f) {
    int best = 0;
    float best_val = -2.0f;
    for (int axis = 0; axis < 6; ++axis) {
        const float c = component(f, axis);
        if (c > best_val) {
            best_val = c;
            best = axis;
        }
    }
    return best;
}

float angle_from_axis(const GravityFilter& f, int axis) {
    float c = component(f, axis);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c) * RAD_TO_DEG;
}

Euler tilt_angles(const GravityFilter& f, int axis) {
    const float v[3] = {f.x, f.y, f.z};
    const int i = axis % 3;
    const float u = component(f, axis);
    const float a = v[(i + 1) % 3];
    const float b = v[(i + 2) % 3];
    Euler e;
    e.roll = std::atan2(b, u) * RAD_TO_DEG;
    e.pitch = std::atan2(-a, std::sqrt(b * b + u * u)) * RAD_TO_DEG;
    return e;
}

bool tilt_exceeds(const Euler& e, float limit_deg) {
    const float roll = e.roll < 0 ? -e.roll : e.roll;
    const float pitch = e.pitch < 0 ? -e.pitch : e.pitch;
    return roll > limit_deg || pitch > limit_deg;
}

}  // namespace navi
