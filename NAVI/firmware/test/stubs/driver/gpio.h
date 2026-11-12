#pragma once
#include "../esp_common.h"
typedef enum {
  GPIO_NUM_0=0, GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4, GPIO_NUM_5,
  GPIO_NUM_12=12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15, GPIO_NUM_16,
  GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21=21, GPIO_NUM_22,
  GPIO_NUM_23, GPIO_NUM_25=25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32=32,
  GPIO_NUM_33, GPIO_NUM_34, GPIO_NUM_35
} gpio_num_t;
typedef enum { GPIO_MODE_DISABLE, GPIO_MODE_INPUT, GPIO_MODE_OUTPUT } gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE, GPIO_PULLUP_ENABLE } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE, GPIO_PULLDOWN_ENABLE } gpio_pulldown_t;
typedef enum { GPIO_INTR_DISABLE, GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE } gpio_int_type_t;
typedef struct {
  uint64_t pin_bit_mask; gpio_mode_t mode; gpio_pullup_t pull_up_en;
  gpio_pulldown_t pull_down_en; gpio_int_type_t intr_type;
} gpio_config_t;
typedef void (*gpio_isr_t)(void *);
esp_err_t gpio_config(const gpio_config_t *cfg);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
int gpio_get_level(gpio_num_t pin);
esp_err_t gpio_install_isr_service(int flags);
esp_err_t gpio_isr_handler_add(gpio_num_t pin, gpio_isr_t fn, void *arg);
#define GPIO_NUM_NC ((gpio_num_t)-1)
typedef enum { GPIO_DRIVE_CAP_0, GPIO_DRIVE_CAP_1, GPIO_DRIVE_CAP_2, GPIO_DRIVE_CAP_3 } gpio_drive_cap_t;
esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode);
esp_err_t gpio_reset_pin(gpio_num_t pin);
esp_err_t gpio_set_drive_capability(gpio_num_t pin, gpio_drive_cap_t strength);
