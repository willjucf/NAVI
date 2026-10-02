// NAVI sensor polling implementation.

#include "sensors.h"

#include <cstring>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "navi_pins.h"

namespace navi {

static const char* TAG = "sensors";

// BNO055 registers (datasheet section 4.2)

static constexpr uint8_t BNO055_ADDR = 0x28;      // 0x29 if ADR is high
static constexpr uint8_t REG_CHIP_ID = 0x00;
static constexpr uint8_t REG_OPR_MODE = 0x3D;
static constexpr uint8_t REG_PWR_MODE = 0x3E;
static constexpr uint8_t REG_SYS_TRIGGER = 0x3F;
static constexpr uint8_t REG_UNIT_SEL = 0x3B;
static constexpr uint8_t REG_EULER_H_LSB = 0x1A;
static constexpr uint8_t REG_CALIB_STAT = 0x35;

static constexpr uint8_t CHIP_ID_VALUE = 0xA0;
static constexpr uint8_t MODE_CONFIG = 0x00;
static constexpr uint8_t MODE_NDOF = 0x0C;        // full fusion, report 3.6.1
static constexpr uint8_t PWR_NORMAL = 0x00;

static i2c_master_bus_handle_t s_bus = nullptr;
static i2c_master_dev_handle_t s_imu = nullptr;
static bool s_imu_ok = false;

static SensorState s_latest;
static portMUX_TYPE s_latest_mux = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t imu_write(uint8_t reg, uint8_t value) {
    const uint8_t payload[2] = {reg, value};
    return i2c_master_transmit(s_imu, payload, sizeof(payload), 100);
}

static esp_err_t imu_read(uint8_t reg, uint8_t* buf, size_t len) {
    return i2c_master_transmit_receive(s_imu, &reg, 1, buf, len, 100);
}

static bool imu_init() {
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = NAVI_I2C_SDA_PIN;
    bus_cfg.scl_io_num = NAVI_I2C_SCL_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed");
        return false;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = BNO055_ADDR;
    dev_cfg.scl_speed_hz = NAVI_I2C_FREQ_HZ;
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_imu) != ESP_OK) {
        ESP_LOGE(TAG, "i2c add device failed");
        return false;
    }

    // The BNO055 needs a moment after power on before it answers, so one
    // failed read is not conclusive. Retry briefly.
    uint8_t chip_id = 0;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (imu_read(REG_CHIP_ID, &chip_id, 1) == ESP_OK &&
            chip_id == CHIP_ID_VALUE) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (chip_id != CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "BNO055 not found, chip id 0x%02X expected 0x%02X. "
                      "Check SDA on GPIO%d and SCL on GPIO%d.",
                 chip_id, CHIP_ID_VALUE, NAVI_I2C_SDA_PIN, NAVI_I2C_SCL_PIN);
        return false;
    }

    // Mode changes only work from config mode, and the datasheet wants a
    // settling delay after each one.
    imu_write(REG_OPR_MODE, MODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(25));
    imu_write(REG_PWR_MODE, PWR_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(10));
    imu_write(REG_SYS_TRIGGER, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    imu_write(REG_UNIT_SEL, 0x00);           // degrees, m/s2, Android layout
    vTaskDelay(pdMS_TO_TICKS(10));

    // NDOF fuses accel, gyro and mag on the chip (report 3.6.1), so we read
    // orientation directly instead of writing our own fusion.
    imu_write(REG_OPR_MODE, MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(25));

    ESP_LOGI(TAG, "BNO055 online in NDOF mode at 0x%02X", BNO055_ADDR);
    return true;
}

// HC SR04

// Longest echo we wait for. A 4 m round trip is about 23 ms, so 30 ms covers
// full range with margin and caps how long a poll can take.
static constexpr int64_t ECHO_TIMEOUT_US = 30000;

static void uss_pin_init(gpio_num_t trig, gpio_num_t echo) {
    gpio_config_t t = {};
    t.pin_bit_mask = (1ULL << trig);
    t.mode = GPIO_MODE_OUTPUT;
    t.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&t);
    gpio_set_level(trig, 0);

    gpio_config_t e = {};
    e.pin_bit_mask = (1ULL << echo);
    e.mode = GPIO_MODE_INPUT;
    // No internal pull. The sensor drives this line itself through a 510 ohm
    // series resistor. See navi_pins.h and docs/wiring.md.
    e.pull_up_en = GPIO_PULLUP_DISABLE;
    e.pull_down_en = GPIO_PULLDOWN_DISABLE;
    e.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&e);
}

// One measurement. Busy waits, which is fine because it is capped at 30 ms and
// runs on a low priority sensor task, never the UART path or an ISR.
static int32_t uss_measure(gpio_num_t trig, gpio_num_t echo) {
    gpio_set_level(trig, 0);
    esp_rom_delay_us(4);
    gpio_set_level(trig, 1);
    esp_rom_delay_us(10);            // 10 us trigger pulse the part needs
    gpio_set_level(trig, 0);

    const int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(echo) == 0) {
        if (esp_timer_get_time() - start_wait > ECHO_TIMEOUT_US) {
            return USS_NO_READING;   // echo never started
        }
    }

    const int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 1) {
        if (esp_timer_get_time() - echo_start > ECHO_TIMEOUT_US) {
            return USS_NO_READING;   // echo never ended, treat as no target
        }
    }

    return echo_us_to_mm(esp_timer_get_time() - echo_start);
}

// Public interface

bool sensors_init() {
    uss_pin_init(NAVI_USS1_TRIG_PIN, NAVI_USS1_ECHO_PIN);
    uss_pin_init(NAVI_USS2_TRIG_PIN, NAVI_USS2_ECHO_PIN);
    ESP_LOGI(TAG, "ultrasonics armed, left trig GPIO%d echo GPIO%d, "
                  "right trig GPIO%d echo GPIO%d",
             NAVI_USS1_TRIG_PIN, NAVI_USS1_ECHO_PIN,
             NAVI_USS2_TRIG_PIN, NAVI_USS2_ECHO_PIN);

    s_imu_ok = imu_init();
    return s_imu_ok;
}

void sensors_poll(SensorState& out) {
    out.imu_ok = s_imu_ok;
    if (s_imu_ok) {
        uint8_t buf[6] = {0};
        if (imu_read(REG_EULER_H_LSB, buf, sizeof(buf)) == ESP_OK) {
            out.euler = decode_euler(buf);
            out.tilt_alarm = tilt_exceeds(out.euler, TILT_LIMIT_DEG);
        } else {
            out.imu_ok = false;      // a failed read is not a zero reading
        }
        uint8_t calib = 0;
        if (imu_read(REG_CALIB_STAT, &calib, 1) == ESP_OK) {
            out.calib = decode_calib(calib);
        }
    }

    // Take turns with a gap between. Two HC SR04s fired together hear each
    // other's burst and report ghost distances (report 3.7.6).
    out.left_mm = uss_measure(NAVI_USS1_TRIG_PIN, NAVI_USS1_ECHO_PIN);
    vTaskDelay(pdMS_TO_TICKS(15));   // let the first burst die away
    out.right_mm = uss_measure(NAVI_USS2_TRIG_PIN, NAVI_USS2_ECHO_PIN);

    const bool left_close = out.left_mm != USS_NO_READING &&
                            out.left_mm < BUMPER_STOP_MM;
    const bool right_close = out.right_mm != USS_NO_READING &&
                             out.right_mm < BUMPER_STOP_MM;
    out.bumper = left_close || right_close;
}

static void sensor_task(void* arg) {
    (void)arg;
    SensorState local;
    while (true) {
        sensors_poll(local);
        taskENTER_CRITICAL(&s_latest_mux);
        s_latest = local;
        taskEXIT_CRITICAL(&s_latest_mux);
        vTaskDelay(pdMS_TO_TICKS(50));        // 20 Hz is plenty for a bumper
    }
}

void sensors_start_task() {
    xTaskCreate(sensor_task, "sensors", 4096, nullptr, 4, nullptr);
}

void sensors_latest(SensorState& out) {
    taskENTER_CRITICAL(&s_latest_mux);
    out = s_latest;
    taskEXIT_CRITICAL(&s_latest_mux);
}

}  // namespace navi
