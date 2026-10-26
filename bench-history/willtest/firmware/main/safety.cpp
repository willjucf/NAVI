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

// Written from the interrupt, read from tasks. volatile plus the atomic width
// is enough here because every write is a whole word store and there is no
// read modify write anywhere in the ISR.
static volatile bool s_estop_latched = false;
static volatile HaltReason s_reason = HaltReason::NONE;
static volatile int64_t s_last_edge_us = 0;

// Debounce only applies to leaving the halt, never to entering it. A contact
// bouncing into a stop is still a stop, and delaying the trigger to filter it
// would eat directly into the 500 ms budget in Table 2.4.1.
static constexpr int64_t DEBOUNCE_US = 20000;   // 20 ms

void IRAM_ATTR motors_cut_from_isr() {
    // GPIO only. Nothing here calls into a driver that might take a lock or
    // live outside IRAM, because this runs from an interrupt that is allowed
    // to fire while the flash cache is disabled.
    //
    // Opening the relay is the stop. It breaks the motor power path outright,
    // which is a stronger guarantee than a duty cycle of zero and is what
    // report 3.9.5 asks for. The PWM registers follow on the control loop next
    // pass, which cannot be more than one 10 ms period away.
    gpio_set_level(NAVI_RELAY_PIN, 0);
    gpio_set_level(NAVI_LED_PIN, 1);
    gpio_set_level(NAVI_BUZZER_PIN, 1);
}

void motors_halt() {
    // Task context only. ledc_set_duty and ledc_update_duty are not interrupt
    // safe, which is the whole reason motors_cut_from_isr exists separately.
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    gpio_set_level(NAVI_RELAY_PIN, 0);
}

void safety_arm() {
    if (s_estop_latched) {
        return;                               // never arm out from under a halt
    }
    gpio_set_level(NAVI_RELAY_PIN, 1);
    set_indicators(false);
}

void set_indicators(bool on) {
    gpio_set_level(NAVI_LED_PIN, on ? 1 : 0);
    gpio_set_level(NAVI_BUZZER_PIN, on ? 1 : 0);
}

// The interrupt itself. Kept as short as it can be: no logging, no allocation,
// no driver calls, no blocking. Everything here either sets a flag or writes a
// GPIO register.
static void IRAM_ATTR estop_isr(void* arg) {
    (void)arg;
    s_last_edge_us = esp_timer_get_time();
    s_estop_latched = true;
    s_reason = HaltReason::ESTOP_BUTTON;
    motors_cut_from_isr();
}

void safety_init() {
    // Outputs first, so the relay and indicators are in a defined state before
    // the interrupt that manipulates them can possibly fire.
    gpio_config_t out = {};
    out.pin_bit_mask = (1ULL << NAVI_RELAY_PIN) |
                       (1ULL << NAVI_LED_PIN) |
                       (1ULL << NAVI_BUZZER_PIN);
    out.mode = GPIO_MODE_OUTPUT;
    out.pull_up_en = GPIO_PULLUP_DISABLE;
    out.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&out);

    // Start halted. The chair does not become drivable until the Pi has
    // handshaked and the loop is actually running, which is the initialization
    // ordering report 6.4 describes.
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

    // If the button is already held, or the test jumper is already in place,
    // start latched rather than waiting for an edge that has already happened.
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
    // Refuse while the pin is still held. Otherwise a reset command issued
    // during a held button would clear the latch and then immediately need
    // another edge to reassert it, leaving a window where the chair is live
    // with the stop still pressed.
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
