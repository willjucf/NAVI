#pragma once
#include "esp_common.h"
// Variadic so argument count mismatches in our own format calls still parse.
#define ESP_LOGI(tag, ...) do { (void)(tag); } while (0)
#define ESP_LOGW(tag, ...) do { (void)(tag); } while (0)
#define ESP_LOGE(tag, ...) do { (void)(tag); } while (0)
#define ESP_INTR_FLAG_IRAM (1 << 9)
