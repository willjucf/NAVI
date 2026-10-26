#pragma once
#include "esp_common.h"
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef struct { int owner; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdTRUE 1
#define pdFALSE 0
void taskENTER_CRITICAL(portMUX_TYPE *mux);
void taskEXIT_CRITICAL(portMUX_TYPE *mux);
