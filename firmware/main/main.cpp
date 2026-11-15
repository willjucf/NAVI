// NAVI ESP32 firmware. The Pi reports hazards; this board decides whether the chair may drive.
// Log output shares UART0 with the Pi link; the parser ignores anything outside <...>.

#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "navi_logic.h"
#include "navi_pins.h"
#include "safety.h"
#include "sensors.h"
#include "wireless.h"
#include "sound.h"

namespace navi {

static const char* TAG = "navi";

static constexpr int UART_RX_BUF = 512;

// 8 bit LEDC maps the 0 to 255 packet range straight onto duty, so what the Pi
// sends is exactly what the motor sees.
static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_8_BIT;
static constexpr uint32_t PWM_FREQ_HZ = 20000;   // above hearing range

// 0.6 m/s^2: 0 to cruise (~139 PWM) in 2 s. Stops skip the ramp.
static constexpr int RAMP_PWM_PER_S = 70;
static constexpr int64_t RAMP_MAX_GAP_US = 50000;   // a late pass ramps at most this much
static constexpr int CONTROL_PERIOD_MS = 10;

struct MotorState {
    int left = 0;
    int right = 0;
};

static MotorState s_motor;              // control task only

// Shared between the UART and control tasks. int64_t isn't atomic on this core, so it's locked.
static DriveCommand s_target;
static int64_t s_last_packet_us = 0;
static bool s_link_established = false;   // at least one valid packet seen
static portMUX_TYPE s_target_mux = portMUX_INITIALIZER_UNLOCKED;

static PacketParser s_parser;           // UART task only

static void publish_command(const DriveCommand& cmd, int64_t when_us) {
    taskENTER_CRITICAL(&s_target_mux);
    s_target = cmd;
    s_last_packet_us = when_us;
    s_link_established = true;
    taskEXIT_CRITICAL(&s_target_mux);
}

static void read_command(DriveCommand& out, int64_t& last_us, bool& established) {
    taskENTER_CRITICAL(&s_target_mux);
    out = s_target;
    last_us = s_last_packet_us;
    established = s_link_established;
    taskEXIT_CRITICAL(&s_target_mux);
}

// Motor output

static void motors_init() {
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = PWM_RESOLUTION;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = PWM_FREQ_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    const gpio_num_t pins[2] = {NAVI_MOTOR_LEFT_PIN, NAVI_MOTOR_RIGHT_PIN};
    for (int i = 0; i < 2; ++i) {
        ledc_channel_config_t ch = {};
        ch.gpio_num = pins[i];
        ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ch.channel = static_cast<ledc_channel_t>(LEDC_CHANNEL_0 + i);
        ch.timer_sel = LEDC_TIMER_0;
        ch.duty = 0;
        ch.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
    ESP_LOGI(TAG, "motor PWM ready, %lu Hz on GPIO%d and GPIO%d",
             (unsigned long)PWM_FREQ_HZ,
             NAVI_MOTOR_LEFT_PIN, NAVI_MOTOR_RIGHT_PIN);
}

static void motors_apply(const MotorState& m) {
    // Sign is direction and will drive the controller direction inputs once
    // wired. Duty is the magnitude so it is always positive.
    const uint32_t left_duty = static_cast<uint32_t>(m.left < 0 ? -m.left : m.left);
    const uint32_t right_duty = static_cast<uint32_t>(m.right < 0 ? -m.right : m.right);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, left_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, right_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// UART receive task

static void uart_init() {
    uart_config_t cfg = {};
    cfg.baud_rate = NAVI_UART_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(NAVI_UART_PORT, UART_RX_BUF,
                                        0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(NAVI_UART_PORT, &cfg));

    // Argument order matters. On the PCB TX is IO16 and RX is IO17, the
    // opposite of most pinout labels. See navi_pins.h.
    ESP_ERROR_CHECK(uart_set_pin(NAVI_UART_PORT,
                                 NAVI_UART_TX_PIN,
                                 NAVI_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "uart%d up at %d baud, tx GPIO%d rx GPIO%d",
             NAVI_UART_PORT, NAVI_UART_BAUD,
             NAVI_UART_TX_PIN, NAVI_UART_RX_PIN);
}

static void uart_rx_task(void* arg) {
    (void)arg;
    uint8_t chunk[64];
    DriveCommand decoded;

    while (true) {
        // Non-blocking read; a pulled cable is caught by the link watchdog.
        const int n = uart_read_bytes(NAVI_UART_PORT, chunk, sizeof(chunk), 0);
        for (int i = 0; i < n; ++i) {
            // Text commands (beep, ussdiag, i2cscan) work over the UART too.
            // The packet parser skips anything outside <...>.
            wireless_uart_text(static_cast<char>(chunk[i]));
            if (s_parser.feed(static_cast<char>(chunk[i]), decoded)) {
                publish_command(decoded, esp_timer_get_time());
            } else if (s_parser.last_error() != ParseError::NONE) {
                // Bad frames are dropped. The last command stands until the watchdog expires.
                ESP_LOGW(TAG, "discarded frame, %d bad of %d total",
                         (int)s_parser.bad_frames(),
                         (int)(s_parser.bad_frames() + s_parser.good_frames()));
            }
        }
        if (n == 0) {
            vTaskDelay(1);            // yield instead of spinning the core
        }
    }
}

// Control loop and both watchdog layers

static void control_task(void* arg) {
    (void)arg;

    // Task watchdog: if this loop stalls for 500 ms the chip resets with the relays open.
    esp_task_wdt_config_t wdt_cfg = {};
    wdt_cfg.timeout_ms = static_cast<uint32_t>(WATCHDOG_TIMEOUT_MS);
    wdt_cfg.idle_core_mask = 0;
    wdt_cfg.trigger_panic = true;
    esp_task_wdt_reconfigure(&wdt_cfg);
    ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));

    SensorState sensors;
    HaltReason last_reason = HaltReason::NONE;
    bool first_pass = true;
    int64_t last_report_us = 0;
    RampClock ramp;

    while (true) {
        const int64_t now = esp_timer_get_time();

        DriveCommand target;
        SafetyInputs safety;
        read_command(target, safety.last_packet_us, safety.link_established);
        sensors_latest(sensors);

        safety.now_us = now;
        // Only the E-stop latches; every other gate is rechecked each pass.
        estop_poll_release();
        safety.estop_latched = estop_active();
        safety.pi_hazard = target.safety_flag;
        safety.bumper = sensors.bumper;
        safety.imu_ok = sensors.imu_ok;
        safety.tilt_alarm = sensors.tilt_alarm;

        const SafetyDecision decision = evaluate_safety(safety);

        if (decision.halted) {
            // No ramp down. A stop is immediate.
            s_motor.left = 0;
            s_motor.right = 0;
            motors_halt();
            set_indicators(true);
        } else {
            if (decision.arm_relay) {
                // Relays only close once the loop is running and packets are arriving.
                safety_arm();
            }
            // A flagged packet never gets here: it halts above.
            const int step = ramp_steps(ramp, now, RAMP_PWM_PER_S, RAMP_MAX_GAP_US);
            if (step > 0) {
                s_motor.left = ramp_toward(s_motor.left, target.left, step);
                s_motor.right = ramp_toward(s_motor.right, target.right, step);
            }
            motors_apply(s_motor);
        }

        if (decision.reason != last_reason || first_pass) {
            if (decision.halted) {
                ESP_LOGW(TAG, "HALTED: %s, pwm 0,0",
                         halt_reason_name(decision.reason));
            } else {
                ESP_LOGI(TAG, "armed and running");
            }
            last_reason = decision.reason;
            first_pass = false;
        }

        if (now - last_report_us > 500000) {     // twice a second
            last_report_us = now;
            ESP_LOGI(TAG,
                     "state=%s pwm=%d,%d target=%d,%d flag=%d "
                     "uss=%ldmm%s tilt=%.1f/%.1f%s frames=%lu/%lu",
                     halt_reason_name(last_reason),
                     s_motor.left, s_motor.right,
                     target.left, target.right, target.safety_flag ? 1 : 0,
                     (long)sensors.uss_mm, sensors.uss_ok ? "" : "(none)",
                     sensors.euler.roll, sensors.euler.pitch,
                     sensors.imu_ok ? "" : "(no imu)",
                     (unsigned long)s_parser.good_frames(),
                     (unsigned long)s_parser.bad_frames());
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

}  // namespace navi

extern "C" void app_main(void) {
    using namespace navi;

    ESP_LOGI(TAG, "NAVI firmware starting");

    // Safety first: relays open and E-stop armed before anything else starts.
    safety_init();
    motors_init();
    motors_halt();
    sensors_init();
    sensors_start_task();
    uart_init();

    // WiFi and BLE packets go through publish_command like UART ones.
    wireless_start(publish_command);

    // Speaker clips and tones, on their own task.
    sound_init();

    // Seed the timestamp. The relays still stay open until a real packet arrives.
    s_last_packet_us = esp_timer_get_time();

    xTaskCreate(uart_rx_task, "uart_rx", 4096, nullptr, 10, nullptr);
    xTaskCreate(control_task, "control", 4096, nullptr, 9, nullptr);

    ESP_LOGI(TAG, "halted until the first valid packet arrives on uart%d, "
             "wifi or ble", NAVI_UART_PORT);
}
