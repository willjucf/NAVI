// Bench test for the ultrasonic on header U1 (IO26 / IO27).
//
// Same pins as firmware/main/navi_pins.h, so a sensor that passes here works on
// the board unchanged. Works out which pin is trigger and which is echo, the
// same way the firmware does, then prints one reading every 200 ms.
//
// RCWL-1655 or HC-SR04, both use the same trigger and echo pulse.

#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr gpio_num_t PIN_A = GPIO_NUM_26;   // U1 pin 2
static constexpr gpio_num_t PIN_B = GPIO_NUM_27;   // U1 pin 3

// 4 m round trip is about 23 ms, so 30 ms covers full range with margin.
static constexpr int64_t ECHO_TIMEOUT_US = 30000;
// No target holds echo high about 48 ms on an RCWL-1655, 38 ms on an HC-SR04.
static constexpr int64_t ECHO_IDLE_WAIT_US = 70000;

static void set_input(gpio_num_t pin) {
    gpio_config_t c = {};
    c.pin_bit_mask = (1ULL << pin);
    c.mode = GPIO_MODE_INPUT;
    c.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&c);
}

// Fraction of 1000 samples over 100 ms where the pin read high.
static int high_percent(gpio_num_t pin) {
    int high = 0;
    for (int i = 0; i < 1000; ++i) {
        high += gpio_get_level(pin);
        esp_rom_delay_us(100);
    }
    return high / 10;
}

// Pulse trig (weak drive, back to input straight after, in case it is really
// the sensor's echo output) and report how long echo went high, or -1.
static int64_t probe(gpio_num_t trig, gpio_num_t echo) {
    set_input(trig);
    set_input(echo);
    gpio_set_drive_capability(trig, GPIO_DRIVE_CAP_0);
    gpio_set_level(trig, 0);
    gpio_set_direction(trig, GPIO_MODE_OUTPUT);
    esp_rom_delay_us(4);
    gpio_set_level(trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig, 0);
    set_input(trig);

    const int64_t start = esp_timer_get_time();
    while (gpio_get_level(echo) == 0) {
        if (esp_timer_get_time() - start > ECHO_TIMEOUT_US) return -1;
    }
    const int64_t rose = esp_timer_get_time();
    while (gpio_get_level(echo) == 1) {
        if (esp_timer_get_time() - rose > ECHO_IDLE_WAIT_US) break;
    }
    return esp_timer_get_time() - rose;
}

extern "C" void app_main() {
    set_input(PIN_A);
    set_input(PIN_B);
    vTaskDelay(pdMS_TO_TICKS(100));  // let the sensor settle after power up

    printf("\n--- diagnostic ---\n");
    printf("GPIO%d idle: high %d%% of the time (expect 0%%)\n", PIN_A, high_percent(PIN_A));
    printf("GPIO%d idle: high %d%% of the time (expect 0%%)\n", PIN_B, high_percent(PIN_B));

    gpio_num_t trig = GPIO_NUM_NC, echo = GPIO_NUM_NC;
    while (trig == GPIO_NUM_NC) {
        const int64_t ab = probe(PIN_A, PIN_B);
        vTaskDelay(pdMS_TO_TICKS(60));
        const int64_t ba = probe(PIN_B, PIN_A);
        printf("pulse GPIO%d -> GPIO%d: %s", PIN_A, PIN_B, ab < 0 ? "no answer\n" : "");
        if (ab >= 0) printf("echo high %lld us\n", (long long)ab);
        printf("pulse GPIO%d -> GPIO%d: %s", PIN_B, PIN_A, ba < 0 ? "no answer\n" : "");
        if (ba >= 0) printf("echo high %lld us\n", (long long)ba);

        if (ab >= 0) { trig = PIN_A; echo = PIN_B; }
        else if (ba >= 0) { trig = PIN_B; echo = PIN_A; }
        else {
            printf("NO SENSOR: check power, GND and the cable. Retrying in 1 s.\n\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    printf("=> TRIG is GPIO%d, ECHO is GPIO%d\n--- end diagnostic ---\n\n", trig, echo);

    gpio_set_drive_capability(trig, GPIO_DRIVE_CAP_2);
    gpio_set_direction(trig, GPIO_MODE_OUTPUT);
    gpio_set_level(trig, 0);

    while (true) {
        gpio_set_level(trig, 1);
        esp_rom_delay_us(10);
        gpio_set_level(trig, 0);

        const int64_t start = esp_timer_get_time();
        bool started = true;
        while (gpio_get_level(echo) == 0) {
            if (esp_timer_get_time() - start > ECHO_TIMEOUT_US) { started = false; break; }
        }
        if (!started) {
            printf("NO ECHO (wiring or power)\n");
        } else {
            const int64_t rose = esp_timer_get_time();
            while (gpio_get_level(echo) == 1 &&
                   esp_timer_get_time() - rose < ECHO_IDLE_WAIT_US) {
            }
            const int64_t width = esp_timer_get_time() - rose;
            if (width > ECHO_TIMEOUT_US) {
                printf("  --- out of range\n");
            } else {
                // Sound covers about 0.343 mm/us, halved for the round trip.
                printf("%6.1f cm\n", width * 0.0343 / 2.0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
