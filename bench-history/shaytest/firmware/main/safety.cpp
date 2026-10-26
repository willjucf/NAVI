// NAVI emergency stop implementation.

#include "safety.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "navi_pins.h"

namespace navi {

static const char* TAG = "safety";

// Written by the ISR, read by tasks. volatile is enough because every write is
// a whole word store and the ISR never does a read modify write.
static volatile bool s_estop_latched = false;
static volatile HaltReason s_reason = HaltReason::NONE;
static volatile int64_t s_last_edge_us = 0;

// Debounce only applies to leaving a halt, never entering one. A bouncing
// contact is still a stop, and delaying it would eat into the 500 ms budget.
static constexpr int64_t DEBOUNCE_US = 20000;   // 20 ms

void IRAM_ATTR motors_cut_from_isr() {
    // GPIO only. This runs from an interrupt that may fire with the flash
    // cache off, so nothing here can take a lock or live outside IRAM.
    //
    // Opening the relay is the stop. It breaks motor power outright, which is
    // stronger than zero duty (report 3.9.5). The control loop zeros the PWM
    // on its next pass, at most 10 ms away.
    gpio_set_level(NAVI_RELAY_PIN, 0);
    gpio_set_level(NAVI_LED_PIN, 1);
    gpio_set_level(NAVI_BUZZER_PIN, 1);
}

void motors_halt() {
    // Task context only. LEDC calls are not interrupt safe, which is why
    // motors_cut_from_isr exists.
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    gpio_set_level(NAVI_RELAY_PIN, 0);
}

void safety_arm() {
    if (s_estop_latched) {
        return;                               // never arm during a halt
    }
    gpio_set_level(NAVI_RELAY_PIN, 1);
    set_indicators(false);
}

void set_indicators(bool on) {
    gpio_set_level(NAVI_LED_PIN, on ? 1 : 0);
    gpio_set_level(NAVI_BUZZER_PIN, on ? 1 : 0);
}

// The ISR. As short as possible: no logging, allocation, driver calls or
// blocking. It only sets flags and writes GPIO.
static void IRAM_ATTR estop_isr(void* arg) {
    (void)arg;
    s_last_edge_us = esp_timer_get_time();
    s_estop_latched = true;
    s_reason = HaltReason::ESTOP_BUTTON;
    motors_cut_from_isr();
}

void safety_init() {
    // Outputs first so relay and indicators are defined before the ISR that
    // drives them can fire.
    gpio_config_t out = {};
    out.pin_bit_mask = (1ULL << NAVI_RELAY_PIN) |
                       (1ULL << NAVI_LED_PIN) |
                       (1ULL << NAVI_BUZZER_PIN);
    out.mode = GPIO_MODE_OUTPUT;
    out.pull_up_en = GPIO_PULLUP_DISABLE;
    out.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&out);

    // Start halted. Nothing drives until the Pi has talked and the loop is
    // running (report 6.4 startup order).
    gpio_set_level(NAVI_RELAY_PIN, 0);
    set_indicators(false);

    gpio_config_t in = {};
    in.pin_bit_mask = (1ULL << NAVI_ESTOP_PIN);
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;      // idles high, a press pulls it low
    in.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&in);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(NAVI_ESTOP_PIN, estop_isr, nullptr);

    // Button already held or test jumper already in at boot. Start latched
    // since that edge already happened and will not fire the ISR.
    if (gpio_get_level(NAVI_ESTOP_PIN) == 0) {
        s_estop_latched = true;
        s_reason = HaltReason::ESTOP_BUTTON;
        ESP_LOGW(TAG, "e stop pin already low at boot, starting halted");
    }

    ESP_LOGI(TAG, "e stop armed on GPIO%d, relay on GPIO%d",
             NAVI_ESTOP_PIN, NAVI_RELAY_PIN);
}

bool estop_active() {
    return s_estop_latched;
}

HaltReason halt_reason() {
    return s_reason;
}

bool safety_reset() {
    // Refuse while the pin is held. Otherwise the latch would clear and the
    // chair would be live with the button still pressed, since no new edge
    // would come to reassert it.
    if (gpio_get_level(NAVI_ESTOP_PIN) == 0) {
        ESP_LOGW(TAG, "reset refused, e stop pin still low");
        return false;
    }
    if (esp_timer_get_time() - s_last_edge_us < DEBOUNCE_US) {
        return false;                         // contact still settling
    }

    s_estop_latched = false;
    s_reason = HaltReason::NONE;
    set_indicators(false);
    gpio_set_level(NAVI_RELAY_PIN, 1);        // close the cutoff relay again
    ESP_LOGI(TAG, "e stop cleared, motors re enabled");
    return true;
}

}  // namespace navi
