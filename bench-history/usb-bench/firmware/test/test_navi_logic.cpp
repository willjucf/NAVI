// Virtual test suite for the NAVI firmware logic core.
//
// Compiles and runs on a host machine with plain g++. No ESP32, no ESP IDF, no
// flashing cycle. That is the reason navi_logic.cpp was kept free of hardware
// dependencies in the first place: the packet parser and the watchdog
// arithmetic are the parts where a bug is most expensive and hardest to see on
// a bench, so they are the parts that get tested exhaustively here.
//
// Build and run:
//   cd firmware/test && make run
//
// Or directly:
//   g++ -std=c++17 -I../main test_navi_logic.cpp ../main/navi_logic.cpp -o t

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "navi_logic.h"

using namespace navi;

// ---------------------------------------------------------------------------
// Minimal test harness
// ---------------------------------------------------------------------------

static int g_checks = 0;
static int g_failures = 0;
static int g_failures_at_start = 0;
static const char* g_current = "";

static void begin(const char* name) {
    g_current = name;
    g_failures_at_start = g_failures;
    printf("  %-56s", name);
}

static void end() {
    printf("%s\n", g_failures == g_failures_at_start ? "ok" : "^^ FAILED");
}

#define CHECK(cond)                                                          \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(cond)) {                                                       \
            ++g_failures;                                                    \
            printf("\n    FAIL %s:%d in %s: %s\n",                           \
                   __FILE__, __LINE__, g_current, #cond);                    \
        }                                                                    \
    } while (0)

#define CHECK_EQ(a, b)                                                       \
    do {                                                                     \
        ++g_checks;                                                          \
        auto va_ = (a);                                                      \
        auto vb_ = (b);                                                      \
        if (!(va_ == vb_)) {                                                 \
            ++g_failures;                                                    \
            printf("\n    FAIL %s:%d in %s: %s == %s (got %lld vs %lld)\n",   \
                   __FILE__, __LINE__, g_current, #a, #b,                    \
                   (long long)va_, (long long)vb_);                          \
        }                                                                    \
    } while (0)

// Push a whole string through the parser, byte at a time, exactly the way the
// UART task does. Returns how many complete valid frames came out.
static int feed_string(PacketParser& p, const std::string& s,
                       DriveCommand& last) {
    int frames = 0;
    DriveCommand tmp;
    for (char c : s) {
        if (p.feed(c, tmp)) {
            last = tmp;
            ++frames;
        }
    }
    return frames;
}

// ---------------------------------------------------------------------------
// Packet parsing
// ---------------------------------------------------------------------------

static void test_parses_a_clean_frame() {
    begin("parses a clean forward command");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<150,150,0>", cmd), 1);
    CHECK_EQ(cmd.left, 150);
    CHECK_EQ(cmd.right, 150);
    CHECK(cmd.safety_flag == false);
    end();
}

static void test_parses_the_stop_packet() {
    // This is the exact packet report 6.13 and the Step 3 test call for.
    begin("parses the stop packet <0,0,1>");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<0,0,1>", cmd), 1);
    CHECK_EQ(cmd.left, 0);
    CHECK_EQ(cmd.right, 0);
    CHECK(cmd.safety_flag == true);
    end();
}

static void test_parses_negative_pwm() {
    begin("parses negative pwm for reverse");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<-255,-120,0>", cmd), 1);
    CHECK_EQ(cmd.left, -255);
    CHECK_EQ(cmd.right, -120);
    end();
}

static void test_ignores_leading_garbage() {
    // Line noise, or a reset part way through a frame, must not poison the
    // next good packet behind it.
    begin("resynchronises after leading garbage");
    PacketParser p;
    DriveCommand cmd;
    // Built with an explicit length because the embedded NUL is part of the
    // point. A bare char* literal would stop there, so the noise bytes after
    // it would never reach the parser and the test would prove nothing.
    std::string noise("xyz!!\x00\xfe\x01", 8);
    noise += "<100,50,0>";
    CHECK_EQ(feed_string(p, noise, cmd), 1);
    CHECK_EQ(cmd.left, 100);
    CHECK_EQ(cmd.right, 50);
    end();
}

static void test_frame_split_across_reads() {
    // The UART hands us whatever happened to arrive, which frequently cuts a
    // frame in half. The parser is byte at a time precisely so this works.
    begin("accepts a frame split across several reads");
    PacketParser p;
    DriveCommand cmd;
    int frames = 0;
    frames += feed_string(p, "<15", cmd);
    frames += feed_string(p, "0,-1", cmd);
    frames += feed_string(p, "50,1>", cmd);
    CHECK_EQ(frames, 1);
    CHECK_EQ(cmd.left, 150);
    CHECK_EQ(cmd.right, -150);
    CHECK(cmd.safety_flag == true);
    end();
}

static void test_truncated_frame_then_recovery() {
    // A start byte inside a frame means the earlier one was truncated. The
    // newer data is the one worth keeping.
    begin("restarts on a start byte inside a frame");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<99,99<10,20,0>", cmd), 1);
    CHECK_EQ(cmd.left, 10);
    CHECK_EQ(cmd.right, 20);
    end();
}

static void test_rejects_missing_field() {
    begin("rejects a frame with only two fields");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<150,150>", cmd), 0);
    CHECK(p.last_error() == ParseError::PACKET_CORRUPTION_ERROR);
    CHECK_EQ(p.bad_frames(), 1u);
    end();
}

static void test_rejects_extra_field() {
    // A four field packet is what a CRC bearing variant would look like. We do
    // not speak that dialect, so it is corruption rather than an extension.
    begin("rejects a frame with four fields");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<150,150,0,77>", cmd), 0);
    CHECK(p.last_error() == ParseError::PACKET_CORRUPTION_ERROR);
    end();
}

static void test_rejects_non_numeric() {
    begin("rejects non numeric payload characters");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<15x,150,0>", cmd), 0);
    CHECK(p.last_error() == ParseError::PACKET_CORRUPTION_ERROR);
    end();
}

static void test_rejects_empty_field() {
    begin("rejects an empty field");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<,150,0>", cmd), 0);
    end();
}

static void test_rejects_bad_flag() {
    begin("rejects a safety flag that is not 0 or 1");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<0,0,7>", cmd), 0);
    end();
}

static void test_oversized_payload() {
    // No stop byte within a plausible distance. The parser must give up rather
    // than run off the end of its buffer.
    begin("abandons an oversized payload without overrunning");
    PacketParser p;
    DriveCommand cmd;
    std::string junk = "<";
    junk.append(MAX_PAYLOAD + 40, '9');
    junk += ">";
    CHECK_EQ(feed_string(p, junk, cmd), 0);
    // last_error() describes the most recent byte, and the trailing junk here
    // clears it again, so the durable signal is the bad frame count.
    CHECK_EQ(p.bad_frames(), 1u);

    // and it still works afterwards
    CHECK_EQ(feed_string(p, "<7,8,0>", cmd), 1);
    CHECK_EQ(cmd.left, 7);
    end();
}

static void test_back_to_back_frames() {
    begin("decodes several frames from one buffer");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<1,2,0><3,4,0><5,6,1>", cmd), 3);
    CHECK_EQ(cmd.left, 5);
    CHECK_EQ(cmd.right, 6);
    CHECK(cmd.safety_flag == true);
    CHECK_EQ(p.good_frames(), 3u);
    end();
}

static void test_out_of_range_pwm_is_clamped() {
    // A structurally valid packet is one the Pi meant to send. Clamping keeps
    // the chair moving within limits, where rejecting would hold the previous
    // command until the watchdog fired instead.
    begin("clamps out of range pwm rather than rejecting");
    PacketParser p;
    DriveCommand cmd;
    CHECK_EQ(feed_string(p, "<9000,-9000,0>", cmd), 1);
    CHECK_EQ(cmd.left, PWM_MAX);
    CHECK_EQ(cmd.right, -PWM_MAX);
    end();
}

static void test_format_round_trip() {
    // What we format must be what we can parse. This is the property that
    // keeps the Pi side and the ESP32 side from drifting apart.
    begin("format and parse round trip");
    char buf[32];
    DriveCommand in;
    in.left = -42;
    in.right = 200;
    in.safety_flag = true;
    const size_t n = format_packet(in, buf, sizeof(buf));
    CHECK(n > 0);
    CHECK(std::string(buf) == "<-42,200,1>");

    PacketParser p;
    DriveCommand out;
    CHECK_EQ(feed_string(p, buf, out), 1);
    CHECK_EQ(out.left, in.left);
    CHECK_EQ(out.right, in.right);
    CHECK(out.safety_flag == in.safety_flag);
    end();
}

// ---------------------------------------------------------------------------
// Watchdog timing
// ---------------------------------------------------------------------------

static void test_watchdog_boundary() {
    // The 500 ms window is a specification number from report 6.13 and Table
    // 2.4.1, so it is tested at the exact boundary rather than approximately.
    begin("watchdog fires at exactly 500 ms and not before");
    const int64_t last = 1000000;
    CHECK(deadline_expired(last + 499000, last, WATCHDOG_TIMEOUT_MS) == false);
    CHECK(deadline_expired(last + 500000, last, WATCHDOG_TIMEOUT_MS) == true);
    CHECK(deadline_expired(last + 501000, last, WATCHDOG_TIMEOUT_MS) == true);
    end();
}

static void test_watchdog_fresh_packet() {
    begin("watchdog stays quiet while packets keep arriving");
    int64_t last = 0;
    for (int64_t t = 0; t < 5000000; t += 50000) {   // a packet every 50 ms
        CHECK(deadline_expired(t, last, WATCHDOG_TIMEOUT_MS) == false);
        last = t;
    }
    end();
}

static void test_watchdog_after_silence() {
    begin("watchdog fires once the stream stops");
    const int64_t last = 2000000;
    CHECK(deadline_expired(last + 250000, last, WATCHDOG_TIMEOUT_MS) == false);
    CHECK(deadline_expired(last + 750000, last, WATCHDOG_TIMEOUT_MS) == true);
    end();
}

// ---------------------------------------------------------------------------
// Safety supervision
// ---------------------------------------------------------------------------

// A chair that is healthy in every respect. Each test spoils exactly one thing,
// so a failure names the condition that caused it.
static SafetyInputs healthy() {
    SafetyInputs in;
    in.now_us = 10000000;
    in.last_packet_us = in.now_us - 50000;   // 50 ms ago, well inside the window
    in.link_established = true;
    in.estop_latched = false;
    in.bumper = false;
    in.imu_ok = true;
    in.tilt_alarm = false;
    return in;
}

static void test_healthy_system_arms() {
    begin("a healthy system arms the relay and drives");
    const SafetyDecision d = evaluate_safety(healthy());
    CHECK(d.halted == false);
    CHECK(d.arm_relay == true);
    CHECK(d.reason == HaltReason::NONE);
    end();
}

static void test_never_arms_before_the_first_packet() {
    // Regression. The firmware used to open the relay at startup and never
    // close it on the normal path, so the chair logged "running" and cycled
    // PWM values while the motor power path stayed physically cut. The
    // opposite mistake is worse: arming during the startup grace window having
    // never once heard from the Pi. Both are silent on a bench.
    begin("never arms before the first packet has arrived");
    SafetyInputs in = healthy();
    in.link_established = false;
    const SafetyDecision d = evaluate_safety(in);
    CHECK(d.halted == true);
    CHECK(d.arm_relay == false);
    CHECK(d.reason == HaltReason::NO_LINK);
    end();
}

static void test_no_link_is_distinct_from_signal_lost() {
    // Different causes, different fixes. Never heard from usually means wiring
    // or a swapped TX and RX; went quiet means a fault mid run.
    begin("never heard from is distinct from went quiet");
    SafetyInputs never = healthy();
    never.link_established = false;
    SafetyInputs quiet = healthy();
    quiet.last_packet_us = quiet.now_us - 900000;
    CHECK(evaluate_safety(never).reason == HaltReason::NO_LINK);
    CHECK(evaluate_safety(quiet).reason == HaltReason::SIGNAL_LOST);
    end();
}

static void test_signal_loss_at_the_boundary() {
    begin("signal loss halts at exactly 500 ms");
    SafetyInputs in = healthy();
    in.last_packet_us = in.now_us - 499000;
    CHECK(evaluate_safety(in).halted == false);
    in.last_packet_us = in.now_us - 500000;
    CHECK(evaluate_safety(in).halted == true);
    CHECK(evaluate_safety(in).reason == HaltReason::SIGNAL_LOST);
    end();
}

static void test_estop_outranks_everything() {
    // Report 6.6. The button is checked before any other gate, so its reason
    // is the one reported even when several conditions are true at once.
    begin("the e stop button outranks every other condition");
    SafetyInputs in = healthy();
    in.estop_latched = true;
    in.link_established = false;
    in.bumper = true;
    in.tilt_alarm = true;
    const SafetyDecision d = evaluate_safety(in);
    CHECK(d.halted == true);
    CHECK(d.arm_relay == false);
    CHECK(d.reason == HaltReason::ESTOP_BUTTON);
    end();
}

static void test_bumper_halts() {
    begin("ultrasonic bumper halts the chair");
    SafetyInputs in = healthy();
    in.bumper = true;
    const SafetyDecision d = evaluate_safety(in);
    CHECK(d.halted == true);
    CHECK(d.reason == HaltReason::OBSTACLE);
    end();
}

static void test_tilt_halts_only_when_the_imu_answers() {
    // A failed I2C read must not read as a level chair, and must not fake a
    // tilt either. The reading is simply not consulted when the IMU is down.
    begin("tilt is trusted only when the imu is actually answering");
    SafetyInputs in = healthy();
    in.imu_ok = true;
    in.tilt_alarm = true;
    CHECK(evaluate_safety(in).reason == HaltReason::TILT);

    in.imu_ok = false;
    CHECK(evaluate_safety(in).halted == false);
    end();
}

static void test_transient_conditions_clear_themselves() {
    // Only the button latches. A chair should not need a manual reset because
    // someone walked past the ultrasonic, and it should resume once the Pi
    // starts talking again.
    begin("bumper and signal loss clear themselves, button does not");
    SafetyInputs in = healthy();

    in.bumper = true;
    CHECK(evaluate_safety(in).halted == true);
    in.bumper = false;
    CHECK(evaluate_safety(in).halted == false);

    in.last_packet_us = in.now_us - 900000;
    CHECK(evaluate_safety(in).halted == true);
    in.last_packet_us = in.now_us - 10000;
    CHECK(evaluate_safety(in).halted == false);

    // The latch is owned by the caller, so it stays set until cleared there.
    in.estop_latched = true;
    CHECK(evaluate_safety(in).halted == true);
    CHECK(evaluate_safety(in).halted == true);
    end();
}

static void test_defaults_are_closed() {
    // A default constructed input must not describe a drivable chair. If a
    // field is ever added and someone forgets to populate it, the failure has
    // to land on the safe side.
    begin("a default constructed input never arms");
    const SafetyInputs in;
    const SafetyDecision d = evaluate_safety(in);
    CHECK(d.halted == true);
    CHECK(d.arm_relay == false);
    end();
}

static void test_halt_reason_names_are_present() {
    begin("every halt reason has a printable name");
    const HaltReason all[] = {HaltReason::NONE, HaltReason::ESTOP_BUTTON,
                              HaltReason::SIGNAL_LOST, HaltReason::TILT,
                              HaltReason::OBSTACLE, HaltReason::NO_LINK};
    for (HaltReason r : all) {
        const char* name = halt_reason_name(r);
        CHECK(name != nullptr);
        CHECK(name[0] != '\0');
    }
    // and they are distinguishable, so a log line is never ambiguous
    CHECK(std::string(halt_reason_name(HaltReason::NO_LINK)) !=
          std::string(halt_reason_name(HaltReason::SIGNAL_LOST)));
    end();
}

// ---------------------------------------------------------------------------
// Motor shaping
// ---------------------------------------------------------------------------

static void test_clamp_pwm() {
    begin("pwm clamps to the documented range");
    CHECK_EQ(clamp_pwm(0), 0);
    CHECK_EQ(clamp_pwm(255), 255);
    CHECK_EQ(clamp_pwm(256), 255);
    CHECK_EQ(clamp_pwm(-255), -255);
    CHECK_EQ(clamp_pwm(-256), -255);
    CHECK_EQ(clamp_pwm(100000), 255);
    end();
}

static void test_ramp_converges() {
    // Report 6.17. The ramp has to actually arrive, and it must not overshoot
    // and oscillate around the target.
    begin("ramp reaches its target without overshooting");
    int current = 0;
    const int target = 200;
    int steps = 0;
    while (current != target && steps < 1000) {
        const int next = ramp_toward(current, target, 12);
        CHECK(next <= target);            // never overshoots upward
        CHECK(next > current);            // always makes progress
        current = next;
        ++steps;
    }
    CHECK_EQ(current, target);
    CHECK(steps < 1000);
    end();
}

static void test_ramp_downward_and_exact() {
    begin("ramp works downward and lands exactly");
    CHECK_EQ(ramp_toward(200, 0, 12), 188);
    CHECK_EQ(ramp_toward(-200, 0, 12), -188);
    CHECK_EQ(ramp_toward(5, 0, 12), 0);       // smaller than a step, snap to it
    CHECK_EQ(ramp_toward(100, 100, 12), 100); // already there
    CHECK_EQ(ramp_toward(0, 200, 0), 200);    // step 0 disables ramping
    end();
}

// ---------------------------------------------------------------------------
// Ultrasonic conversion
// ---------------------------------------------------------------------------

static void test_echo_conversion() {
    begin("echo pulse converts to a sane distance");
    // 1 m round trip is about 5831 us at 343 m/s.
    const int32_t mm = echo_us_to_mm(5831);
    CHECK(mm > 950 && mm < 1050);
    end();
}

static void test_echo_no_reading() {
    // A timeout must be distinguishable from a distance of zero, which would
    // otherwise read as an obstacle pressed right against the sensor.
    begin("timeout and out of range report no reading");
    CHECK_EQ(echo_us_to_mm(0), USS_NO_READING);
    CHECK_EQ(echo_us_to_mm(-1), USS_NO_READING);
    CHECK_EQ(echo_us_to_mm(50), USS_NO_READING);       // closer than the minimum
    CHECK_EQ(echo_us_to_mm(1000000), USS_NO_READING);  // far past the maximum
    end();
}

static void test_echo_monotonic() {
    begin("longer echo always means greater distance");
    int32_t previous = 0;
    for (int64_t us = 200; us < 20000; us += 137) {
        const int32_t mm = echo_us_to_mm(us);
        if (mm == USS_NO_READING) {
            continue;
        }
        CHECK(mm >= previous);
        previous = mm;
    }
    end();
}

// ---------------------------------------------------------------------------
// BNO055 decode
// ---------------------------------------------------------------------------

static void test_decode_euler_zero() {
    begin("decodes an all zero imu reading");
    const uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    const Euler e = decode_euler(buf);
    CHECK(std::fabs(e.heading) < 0.001f);
    CHECK(std::fabs(e.roll) < 0.001f);
    CHECK(std::fabs(e.pitch) < 0.001f);
    end();
}

static void test_decode_euler_known_values() {
    // The BNO055 reports 1/16 of a degree per count, little endian.
    // 0x0B40 is 2880, which is 180.0 degrees.
    begin("decodes a known heading of 180 degrees");
    const uint8_t buf[6] = {0x40, 0x0B, 0x00, 0x00, 0x00, 0x00};
    const Euler e = decode_euler(buf);
    CHECK(std::fabs(e.heading - 180.0f) < 0.01f);
    end();
}

static void test_decode_euler_negative() {
    // Sign handling is the part most likely to be wrong, and a roll that flips
    // sign incorrectly would defeat the tilt check entirely.
    begin("decodes negative angles correctly");
    // -160 counts is -10.0 degrees, which is 0xFF60 little endian.
    const uint8_t buf[6] = {0x00, 0x00, 0x60, 0xFF, 0x60, 0xFF};
    const Euler e = decode_euler(buf);
    CHECK(std::fabs(e.roll + 10.0f) < 0.01f);
    CHECK(std::fabs(e.pitch + 10.0f) < 0.01f);
    end();
}

static void test_decode_calib() {
    begin("decodes the calibration status register");
    CalibStatus c = decode_calib(0xFF);
    CHECK_EQ(c.system, 3);
    CHECK_EQ(c.gyro, 3);
    CHECK_EQ(c.accel, 3);
    CHECK_EQ(c.mag, 3);

    c = decode_calib(0x00);
    CHECK_EQ(c.system, 0);
    CHECK_EQ(c.mag, 0);

    // 0b11 00 01 10: system 3, gyro 0, accel 1, mag 2
    c = decode_calib(0xC6);
    CHECK_EQ(c.system, 3);
    CHECK_EQ(c.gyro, 0);
    CHECK_EQ(c.accel, 1);
    CHECK_EQ(c.mag, 2);
    end();
}

static void test_tilt_detection() {
    begin("tilt alarm triggers on roll or pitch, either sign");
    Euler e;
    e.roll = 5.0f;
    e.pitch = 5.0f;
    CHECK(tilt_exceeds(e, 15.0f) == false);

    e.roll = 20.0f;
    CHECK(tilt_exceeds(e, 15.0f) == true);

    e.roll = -20.0f;
    CHECK(tilt_exceeds(e, 15.0f) == true);

    e.roll = 0.0f;
    e.pitch = -30.0f;
    CHECK(tilt_exceeds(e, 15.0f) == true);

    // heading is a compass bearing, not a tilt, so it must never trigger
    e.pitch = 0.0f;
    e.heading = 359.0f;
    CHECK(tilt_exceeds(e, 15.0f) == false);
    end();
}

// ---------------------------------------------------------------------------

int main() {
    printf("\nNAVI firmware logic suite\n");
    printf("=========================\n\n");

    printf("packet parsing (report 6.13)\n");
    test_parses_a_clean_frame();
    test_parses_the_stop_packet();
    test_parses_negative_pwm();
    test_ignores_leading_garbage();
    test_frame_split_across_reads();
    test_truncated_frame_then_recovery();
    test_rejects_missing_field();
    test_rejects_extra_field();
    test_rejects_non_numeric();
    test_rejects_empty_field();
    test_rejects_bad_flag();
    test_oversized_payload();
    test_back_to_back_frames();
    test_out_of_range_pwm_is_clamped();
    test_format_round_trip();

    printf("\nwatchdog timing (report 6.13, 6.16)\n");
    test_watchdog_boundary();
    test_watchdog_fresh_packet();
    test_watchdog_after_silence();

    printf("\nsafety supervision (report 6.4, 6.6, 6.13, 6.15)\n");
    test_healthy_system_arms();
    test_never_arms_before_the_first_packet();
    test_no_link_is_distinct_from_signal_lost();
    test_signal_loss_at_the_boundary();
    test_estop_outranks_everything();
    test_bumper_halts();
    test_tilt_halts_only_when_the_imu_answers();
    test_transient_conditions_clear_themselves();
    test_defaults_are_closed();
    test_halt_reason_names_are_present();

    printf("\nmotor shaping (report 6.17)\n");
    test_clamp_pwm();
    test_ramp_converges();
    test_ramp_downward_and_exact();

    printf("\nultrasonic (report 3.7.6)\n");
    test_echo_conversion();
    test_echo_no_reading();
    test_echo_monotonic();

    printf("\nBNO055 decode (report 3.6)\n");
    test_decode_euler_zero();
    test_decode_euler_known_values();
    test_decode_euler_negative();
    test_decode_calib();
    test_tilt_detection();

    printf("\n=========================\n");
    if (g_failures == 0) {
        printf("PASS  %d checks\n\n", g_checks);
        return 0;
    }
    printf("FAIL  %d of %d checks failed\n\n", g_failures, g_checks);
    return 1;
}
