// Speaker output on IO5 (LM386 on the aux board). Tones are a square wave; clips play as
// 8-bit samples on a 78 kHz PWM. LEDC timer 1 / channel 2, never the motor PWM.

#include "sound.h"

#include <cstdint>
#include <cstring>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "navi_pins.h"

namespace navi {

static const char* TAG = "sound";

static constexpr ledc_mode_t MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t TIMER = LEDC_TIMER_1;
static constexpr ledc_channel_t CHANNEL = LEDC_CHANNEL_2;
static constexpr uint32_t TONE_DUTY = 512;      // 50% of 10 bit
static constexpr uint32_t VOICE_PWM_HZ = 78125; // 8 bit duty, above hearing
static constexpr uint32_t TIMER_HZ = 4000000;   // 0.25 us ticks: 8 and 16 kHz divide exactly

// Clips made by tools/make_sounds.py and embedded by main/CMakeLists.txt.
#define NAVI_CLIP(name)                                                     \
    extern const uint8_t name##_start[] asm("_binary_" #name "_raw_start"); \
    extern const uint8_t name##_end[] asm("_binary_" #name "_raw_end");
NAVI_CLIP(estop)
NAVI_CLIP(obstacle)
NAVI_CLIP(red)
NAVI_CLIP(tilt)
NAVI_CLIP(link)
NAVI_CLIP(resume)
#ifdef NAVI_HAS_VOICE
NAVI_CLIP(voice)
#endif
#undef NAVI_CLIP

struct Clip {
    const char* name;
    const uint8_t* start;
    const uint8_t* end;
    uint32_t rate;                   // samples per second
};

static const Clip CLIPS[] = {
    {"estop", estop_start, estop_end, 8000},          // tools/make_sounds.py
    {"obstacle", obstacle_start, obstacle_end, 8000},
    {"red", red_start, red_end, 8000},
    {"tilt", tilt_start, tilt_end, 8000},
    {"link", link_start, link_end, 8000},
    {"resume", resume_start, resume_end, 8000},
#ifdef NAVI_HAS_VOICE
    {"voice", voice_start, voice_end, 16000},  // optional 16 kHz recording
#endif
};
static constexpr int NUM_CLIPS = sizeof(CLIPS) / sizeof(CLIPS[0]);

// Requests for the sound task: a clip index, or one of these.
static constexpr int REQ_TONES = -1;
static constexpr int REQ_ALL = -2;

static QueueHandle_t s_queue = nullptr;
static volatile bool s_busy = false;
static gptimer_handle_t s_timer = nullptr;

// Voice playback state, shared with the sample interrupt.
static const uint8_t* volatile s_pos = nullptr;
static const uint8_t* volatile s_end = nullptr;
static volatile bool s_playing = false;

static bool setup_pin(ledc_timer_bit_t bits, uint32_t hz) {
    ledc_timer_config_t t = {};
    t.speed_mode = MODE;
    t.duty_resolution = bits;
    t.timer_num = TIMER;
    t.freq_hz = hz;
    t.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&t) != ESP_OK) {
        ESP_LOGW(TAG, "LEDC timer setup failed at %lu Hz", (unsigned long)hz);
        return false;
    }
    ledc_channel_config_t c = {};
    c.gpio_num = NAVI_BUZZER_PIN;
    c.speed_mode = MODE;
    c.channel = CHANNEL;
    c.timer_sel = TIMER;
    c.duty = 0;
    c.hpoint = 0;
    return ledc_channel_config(&c) == ESP_OK;
}

// Give the pin back to plain GPIO, as speaker_tone does.
static void release_pin() {
    ledc_stop(MODE, CHANNEL, 0);
    gpio_reset_pin(NAVI_BUZZER_PIN);
    gpio_set_direction(NAVI_BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(NAVI_BUZZER_PIN, 0);
}

static void set_duty(uint32_t duty) {
    ledc_set_duty(MODE, CHANNEL, duty);
    ledc_update_duty(MODE, CHANNEL);
}

// Tones

static void tone_for(uint32_t hz, int ms) {
    ledc_set_freq(MODE, TIMER, hz);
    set_duty(TONE_DUTY);
    vTaskDelay(pdMS_TO_TICKS(ms));
    set_duty(0);
}

static void play_tones() {
    if (!setup_pin(LEDC_TIMER_10_BIT, 1000)) {
        return;
    }
    const uint32_t steady[3] = {440, 1000, 2000};
    for (uint32_t hz : steady) {
        ESP_LOGI(TAG, "tone %lu Hz, 1 s", (unsigned long)hz);
        tone_for(hz, 1000);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_LOGI(TAG, "sweep 200 to 4000 Hz");
    for (uint32_t hz = 200; hz <= 4000; hz += 50) {
        ledc_set_freq(MODE, TIMER, hz);
        set_duty(TONE_DUTY);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    set_duty(0);
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "three beeps, 1500 Hz");
    for (int i = 0; i < 3; ++i) {
        tone_for(1500, 150);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    release_pin();
}

// Voice

// Once per sample while a clip plays: the next sample becomes the duty.
static bool on_sample(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
    if (!s_playing) {
        return false;
    }
    const uint8_t* pos = s_pos;
    if (pos >= s_end) {
        s_playing = false;
        set_duty(0);                 // idle with the transistor off
        return false;
    }
    set_duty(*pos);
    s_pos = pos + 1;
    return false;
}

static void play_clip(const Clip& clip) {
    const size_t len = clip.end - clip.start;
    ESP_LOGI(TAG, "say %s (%.1f s)", clip.name, len / (float)clip.rate);
    if (!setup_pin(LEDC_TIMER_8_BIT, VOICE_PWM_HZ)) {
        return;
    }
    gptimer_alarm_config_t ac = {};
    ac.alarm_count = TIMER_HZ / clip.rate;
    ac.reload_count = 0;
    ac.flags.auto_reload_on_alarm = true;
    gptimer_set_alarm_action(s_timer, &ac);
    gptimer_set_raw_count(s_timer, 0);
    s_pos = clip.start;
    s_end = clip.end;
    s_playing = true;
    gptimer_start(s_timer);
    while (s_playing) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    gptimer_stop(s_timer);
    release_pin();
}

static void sound_task(void* arg) {
    (void)arg;
    int req = 0;
    while (true) {
        if (xQueueReceive(s_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (req == REQ_TONES) {
            play_tones();
        } else if (req == REQ_ALL) {
            for (const Clip& clip : CLIPS) {
                play_clip(clip);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        } else if (req >= 0 && req < NUM_CLIPS) {
            play_clip(CLIPS[req]);
        }
        ESP_LOGI(TAG, "done");
        s_busy = false;
    }
}

void sound_init() {
    gptimer_config_t tc = {};
    tc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    tc.direction = GPTIMER_COUNT_UP;
    tc.resolution_hz = TIMER_HZ;
    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = on_sample;
    // The alarm period is set per clip in play_clip.
    // A failure here only loses the voice clips. Never stop the boot for it.
    if (gptimer_new_timer(&tc, &s_timer) != ESP_OK ||
        gptimer_register_event_callbacks(s_timer, &cbs, nullptr) != ESP_OK ||
        gptimer_enable(s_timer) != ESP_OK) {
        ESP_LOGE(TAG, "sample timer setup failed, no voice clips");
        s_timer = nullptr;
    }
    s_queue = xQueueCreate(2, sizeof(int));
    xTaskCreate(sound_task, "sound", 3072, nullptr, 3, nullptr);
    ESP_LOGI(TAG, "ready: tones, say estop|obstacle|red|tilt|link|resume|voice|all");
}

static bool request(int req) {
    if (s_queue == nullptr) {
        return false;
    }
    if (s_busy) {
        ESP_LOGW(TAG, "busy, a sound is already playing");
        return false;
    }
    s_busy = true;
    xQueueSend(s_queue, &req, 0);
    return true;
}

bool sound_tones() {
    return request(REQ_TONES);
}

bool sound_say(const char* name) {
    if (s_timer == nullptr) {
        ESP_LOGW(TAG, "no voice clips, the sample timer failed at boot");
        return false;
    }
    if (strcmp(name, "all") == 0) {
        return request(REQ_ALL);
    }
    for (int i = 0; i < NUM_CLIPS; ++i) {
        if (strcmp(name, CLIPS[i].name) == 0) {
            return request(i);
        }
    }
    ESP_LOGW(TAG, "no clip \"%s\" (estop, obstacle, red, tilt, link, resume, voice "
             "or all)", name);
    return false;
}

bool sound_busy() {
    return s_busy;
}

}  // namespace navi
