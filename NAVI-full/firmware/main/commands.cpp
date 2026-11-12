// Text commands over the UART. See commands.h.

#include "commands.h"

#include <cstddef>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "navi_logic.h"
#include "safety.h"
#include "sensors.h"

namespace navi {

static const char* TAG = "cmd";

// Text commands: bytes outside <...>. Each runs in its own task.

struct TextCommand {
    char line[24] = {0};
    size_t len = 0;
    bool in_packet = false;

    void feed(char c);
};

static void i2cscan_task(void* arg) {
    (void)arg;
    sensors_i2c_scan();
    vTaskDelete(nullptr);
}

static void beep_task(void* arg) {
    (void)arg;
    // Two pitches, so a dead speaker and a muffled one sound different.
    speaker_tone(1000, 500);
    speaker_tone(2000, 500);
    ESP_LOGI(TAG, "beep done");
    vTaskDelete(nullptr);
}

static void run_command(const char* line) {
    if (strcmp(line, "ussdiag") == 0) {
        ESP_LOGI(TAG, "running ussdiag");
        sensors_request_uss_diag();
    } else if (strcmp(line, "i2cscan") == 0) {
        ESP_LOGI(TAG, "running i2cscan");
        xTaskCreate(i2cscan_task, "i2cscan", 4096, nullptr, 3, nullptr);
    } else if (strcmp(line, "beep") == 0) {
        ESP_LOGI(TAG, "beep: 1 kHz then 2 kHz on the speaker");
        xTaskCreate(beep_task, "beep", 3072, nullptr, 3, nullptr);
    } else if (line[0] != 0) {
        ESP_LOGW(TAG, "unknown command \"%s\" (try i2cscan, ussdiag or beep)", line);
    }
}

void TextCommand::feed(char c) {
    if (c == START_BYTE) {
        in_packet = true;
        len = 0;
    } else if (c == STOP_BYTE) {
        in_packet = false;
        len = 0;
    } else if (in_packet) {
        // part of a drive packet, not a command
    } else if (c == '\n' || c == '\r') {
        line[len] = 0;
        run_command(line);
        len = 0;
    } else if (len < sizeof(line) - 1) {
        line[len++] = c;
    }
}

static TextCommand s_uart_text;      // UART receive task only

void commands_uart_text(char c) {
    s_uart_text.feed(c);
}

}  // namespace navi
