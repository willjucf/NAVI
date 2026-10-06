// Bench test for the two HC SR04s.
//
// Same pins as firmware/main/navi_pins.h, so sensors that pass here work on
// the board unchanged. Fires them in turn, never together, because two HC SR04s
// hear each other's burst. Prints one line every 200 ms.

#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct Sensor {
    const char* name;
    gpio_num_t trig;
    gpio_num_t echo;
};

static constexpr Sensor SENSORS[] = {
    {"left ", GPIO_NUM_18, GPIO_NUM_19},
    {"right", GPIO_NUM_27, GPIO_NUM_26},
};

// 4 m round trip is about 23 ms, so 30 ms covers full range with margin.
static constexpr int64_t ECHO_TIMEOUT_US = 30000;

enum class Result { OK, NO_ECHO_START, OUT_OF_RANGE };

static void sensor_init(const Sensor& s) {
    gpio_config_t t = {};
    t.pin_bit_mask = (1ULL << s.trig);
    t.mode = GPIO_MODE_OUTPUT;
    gpio_config(&t);
    gpio_set_level(s.trig, 0);

    gpio_config_t e = {};
    e.pin_bit_mask = (1ULL << s.echo);
    e.mode = GPIO_MODE_INPUT;
    gpio_config(&e);
}

static Result measure(const Sensor& s, int64_t& echo_us) {
    gpio_set_level(s.trig, 0);
    esp_rom_delay_us(4);
    gpio_set_level(s.trig, 1);
    esp_rom_delay_us(10);            // 10 us trigger pulse the part needs
    gpio_set_level(s.trig, 0);

    const int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(s.echo) == 0) {
        if (esp_timer_get_time() - start_wait > ECHO_TIMEOUT_US) {
            return Result::NO_ECHO_START;
        }
    }

    // With no target the sensor holds ECHO high for about 38 ms. Wait it out
    // so the next trigger is not ignored.
    const int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(s.echo) == 1) {
        if (esp_timer_get_time() - echo_start > ECHO_TIMEOUT_US) {
            while (gpio_get_level(s.echo) == 1 &&
                   esp_timer_get_time() - echo_start < 60000) {
            }
            return Result::OUT_OF_RANGE;
        }
    }

    echo_us = esp_timer_get_time() - echo_start;
    return Result::OK;
}

static void print_reading(const Sensor& s, Result r, int64_t echo_us) {
    printf("%s ", s.name);
    switch (r) {
        case Result::OK:
            // Sound covers about 0.343 mm/us, halved for the round trip.
            printf("%6.1f cm      ", echo_us * 0.0343 / 2.0);
            break;
        case Result::NO_ECHO_START:
            printf("NO ECHO (wiring)");
            break;
        case Result::OUT_OF_RANGE:
            printf("  --- out of range");
            break;
    }
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

// Pulse one TRIG and report how long each ECHO pin went high. Shows swapped
// or crossed wires as well as dead sensors.
static void diagnose() {
    printf("\n--- diagnostic ---\n");
    for (const Sensor& s : SENSORS) {
        printf("ECHO GPIO%d idle: high %d%% of the time (expect 0%%)\n",
               s.echo, high_percent(s.echo));
    }
    for (const Sensor& t : SENSORS) {
        gpio_set_level(t.trig, 1);
        esp_rom_delay_us(10);
        gpio_set_level(t.trig, 0);
        int64_t rose[2] = {-1, -1};
        int64_t fell[2] = {-1, -1};
        const int64_t start = esp_timer_get_time();
        while (esp_timer_get_time() - start < 80000) {
            const int64_t now = esp_timer_get_time() - start;
            for (int i = 0; i < 2; ++i) {
                const int lvl = gpio_get_level(SENSORS[i].echo);
                if (lvl && rose[i] < 0) rose[i] = now;
                if (!lvl && rose[i] >= 0 && fell[i] < 0) fell[i] = now;
            }
        }
        for (int i = 0; i < 2; ++i) {
            printf("pulse TRIG GPIO%d -> ECHO GPIO%d: ", t.trig, SENSORS[i].echo);
            if (rose[i] < 0) {
                printf("never went high\n");
            } else if (fell[i] < 0) {
                printf("high from %lld us and never fell\n", (long long)rose[i]);
            } else {
                printf("high for %lld us\n", (long long)(fell[i] - rose[i]));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("--- end diagnostic ---\n\n");
}

extern "C" void app_main() {
    for (const Sensor& s : SENSORS) {
        sensor_init(s);
        printf("\n%s TRIG GPIO%d, ECHO GPIO%d", s.name, s.trig, s.echo);
    }
    printf("\n");
    vTaskDelay(pdMS_TO_TICKS(100));  // let the sensors settle after power up
    diagnose();

    while (true) {
        for (const Sensor& s : SENSORS) {
            int64_t echo_us = 0;
            const Result r = measure(s, echo_us);
            print_reading(s, r, echo_us);
            printf("   |   ");
            vTaskDelay(pdMS_TO_TICKS(15));   // let the burst die away
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
