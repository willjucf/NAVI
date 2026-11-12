// NAVI sensor polling implementation.

#include "sensors.h"

#include <cstdio>
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

// ICM-20948 registers (datasheet section 7). Bank 0 unless noted.

static constexpr uint8_t ICM_ADDR_AD0_LOW = 0x68;
static constexpr uint8_t ICM_ADDR_AD0_HIGH = 0x69;
static constexpr uint8_t REG_WHO_AM_I = 0x00;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x06;
static constexpr uint8_t REG_PWR_MGMT_2 = 0x07;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x2D;    // 12 bytes, accel then gyro
static constexpr uint8_t REG_BANK_SEL = 0x7F;        // any bank, value bank << 4
static constexpr uint8_t REG_GYRO_CONFIG_1 = 0x01;   // bank 2
static constexpr uint8_t REG_ACCEL_CONFIG = 0x14;    // bank 2

static constexpr uint8_t WHO_AM_I_VALUE = 0xEA;

// DLPF ~50 Hz, 500 dps, 4 g (2 g clips on kerb bumps).
static constexpr uint8_t GYRO_CONFIG_500DPS_DLPF3 = (3 << 3) | (1 << 1) | 1;
static constexpr uint8_t ACCEL_CONFIG_4G_DLPF3 = (3 << 3) | (1 << 1) | 1;
static constexpr float GYRO_LSB_PER_DPS = 65.5f;
static constexpr float ACCEL_LSB_PER_G = 8192.0f;

// Complementary filter: 2% accelerometer per update, and ignore the
// accelerometer while it reads more than 0.15 g away from 1 g (bumps, braking).
static constexpr float ACCEL_WEIGHT = 0.02f;
static constexpr float ACCEL_REJECT_G = 0.15f;

// Readings over the first half second pick the up axis.
static constexpr int64_t IMU_SETTLE_US = 500000;

static i2c_master_bus_handle_t s_bus = nullptr;
static i2c_master_dev_handle_t s_imu = nullptr;
static uint8_t s_imu_addr = 0;
static bool s_imu_ok = false;

static GravityFilter s_gravity;   // sensor task only
static int s_up_axis = -1;        // -1 until chosen
static int64_t s_imu_start_us = 0;
static int64_t s_imu_last_us = 0;

static BumperLatch s_bumper;      // sensor task only

static SensorState s_latest;
static portMUX_TYPE s_latest_mux = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t imu_write(uint8_t reg, uint8_t value) {
    const uint8_t payload[2] = {reg, value};
    return i2c_master_transmit(s_imu, payload, sizeof(payload), 100);
}

static esp_err_t imu_read(uint8_t reg, uint8_t* buf, size_t len) {
    return i2c_master_transmit_receive(s_imu, &reg, 1, buf, len, 100);
}

static esp_err_t imu_bank(uint8_t bank) {
    return imu_write(REG_BANK_SEL, static_cast<uint8_t>(bank << 4));
}

// Look for WHO_AM_I at one address. The chip needs a moment after power on, so
// one failed read is not conclusive.
static bool imu_probe(uint8_t addr) {
    if (s_imu) {
        i2c_master_bus_rm_device(s_imu);
        s_imu = nullptr;
    }
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = NAVI_I2C_FREQ_HZ;
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_imu) != ESP_OK) {
        return false;
    }
    for (int attempt = 0; attempt < 5; ++attempt) {
        uint8_t id = 0;
        imu_bank(0);
        if (imu_read(REG_WHO_AM_I, &id, 1) == ESP_OK) {
            if (id == WHO_AM_I_VALUE) {
                return true;
            }
            ESP_LOGW(TAG, "0x%02X answered with WHO_AM_I 0x%02X, expected 0x%02X",
                     addr, id, WHO_AM_I_VALUE);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}

static bool imu_init() {
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = NAVI_I2C_SDA_PIN;
    bus_cfg.scl_io_num = NAVI_I2C_SCL_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    // The PCB has no I2C pull ups, so the internal ones are all there is.
    bus_cfg.flags.enable_internal_pullup = true;
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed");
        return false;
    }

    // AD0 picks the address and was left floating on the reviewed PCB, so try
    // both.
    if (imu_probe(ICM_ADDR_AD0_LOW)) {
        s_imu_addr = ICM_ADDR_AD0_LOW;
    } else if (imu_probe(ICM_ADDR_AD0_HIGH)) {
        s_imu_addr = ICM_ADDR_AD0_HIGH;
    } else {
        ESP_LOGE(TAG, "ICM-20948 not found at 0x68 or 0x69. Check SDA on "
                      "GPIO%d, SCL on GPIO%d, and the IMU's VDDIO.",
                 NAVI_I2C_SDA_PIN, NAVI_I2C_SCL_PIN);
        return false;
    }

    // Reset, then wake with the best available clock. The chip boots asleep.
    imu_bank(0);
    imu_write(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(20));
    imu_bank(0);
    imu_write(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    imu_write(REG_PWR_MGMT_2, 0x00);           // accel and gyro on

    imu_bank(2);
    imu_write(REG_GYRO_CONFIG_1, GYRO_CONFIG_500DPS_DLPF3);
    imu_write(REG_ACCEL_CONFIG, ACCEL_CONFIG_4G_DLPF3);
    imu_bank(0);                                // data registers live here
    vTaskDelay(pdMS_TO_TICKS(20));

    s_imu_start_us = esp_timer_get_time();
    s_imu_last_us = s_imu_start_us;
    ESP_LOGI(TAG, "ICM-20948 online at 0x%02X", s_imu_addr);
    return true;
}

static void imu_poll(SensorState& out) {
    out.imu_ok = false;
    out.tilt_alarm = false;
    if (!s_imu_ok) {
        return;
    }

    uint8_t buf[12] = {0};
    if (imu_read(REG_ACCEL_XOUT_H, buf, sizeof(buf)) != ESP_OK) {
        return;                     // a failed read is not a zero reading
    }
    const int64_t now = esp_timer_get_time();
    const float dt = (now - s_imu_last_us) / 1e6f;
    s_imu_last_us = now;

    const ImuSample sample = decode_icm_sample(buf, ACCEL_LSB_PER_G,
                                               GYRO_LSB_PER_DPS);
    gravity_filter_update(s_gravity, sample, dt, ACCEL_WEIGHT, ACCEL_REJECT_G);
    if (!s_gravity.ready) {
        return;
    }

    // Let the filter settle, then lock in which way is up. Until then the IMU
    // is not trusted, which evaluate_safety treats as "ignore tilt".
    if (s_up_axis < 0) {
        if (now - s_imu_start_us < IMU_SETTLE_US) {
            return;
        }
        static const char* NAMES[6] = {"+X", "+Y", "+Z", "-X", "-Y", "-Z"};
        s_up_axis = up_axis(s_gravity);
        const float off = angle_from_axis(s_gravity, s_up_axis);
        ESP_LOGI(TAG, "IMU up axis %s, board %.1f deg off square at boot",
                 NAMES[s_up_axis], off);
        if (off > 5.0f) {
            ESP_LOGW(TAG, "IMU is %.1f deg off square. Either the chair "
                          "booted on a slope or the board is mounted at an "
                          "angle; tilt is measured from the axis, not from "
                          "the boot pose.", off);
        }
    }

    out.imu_ok = true;
    out.euler = tilt_angles(s_gravity, s_up_axis);
    out.tilt_deg = angle_from_axis(s_gravity, s_up_axis);
    out.tilt_alarm = tilt_exceeds(out.euler, TILT_LIMIT_DEG);
}

// Ultrasonic
//
// 30 ms covers a 4 m round trip (~23 ms).
static constexpr int64_t ECHO_TIMEOUT_US = 30000;

// With no target an RCWL-1655 holds echo high for about 48 ms (an HC-SR04 for
// about 38 ms). A trigger sent before echo falls is ignored, so wait it out.
static constexpr int64_t ECHO_IDLE_WAIT_US = 70000;

static gpio_num_t s_trig = GPIO_NUM_NC;
static gpio_num_t s_echo = GPIO_NUM_NC;
static int s_uss_retry = 0;

static void uss_set_input(gpio_num_t pin) {
    gpio_config_t c = {};
    c.pin_bit_mask = (1ULL << pin);
    c.mode = GPIO_MODE_INPUT;
    // Pull down so an unplugged sensor reads as no reading, not a phantom echo.
    c.pull_up_en = GPIO_PULLUP_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_ENABLE;
    c.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&c);
}

static bool wait_level(gpio_num_t pin, int level, int64_t timeout_us) {
    const int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if (esp_timer_get_time() - start > timeout_us) {
            return false;
        }
    }
    return true;
}

// Work out which pin is trigger: echo always rises after a trigger. Weakest drive while guessing.
static bool uss_try(gpio_num_t trig, gpio_num_t echo) {
    uss_set_input(trig);
    uss_set_input(echo);
    if (!wait_level(echo, 0, ECHO_IDLE_WAIT_US)) {
        return false;               // echo stuck high, not this way round
    }

    gpio_set_drive_capability(trig, GPIO_DRIVE_CAP_0);
    gpio_set_level(trig, 0);
    gpio_set_direction(trig, GPIO_MODE_OUTPUT);
    esp_rom_delay_us(4);
    gpio_set_level(trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig, 0);
    uss_set_input(trig);

    if (!wait_level(echo, 1, ECHO_TIMEOUT_US)) {
        return false;
    }
    wait_level(echo, 0, ECHO_IDLE_WAIT_US);
    return true;
}

static bool uss_detect() {
    const gpio_num_t a = NAVI_USS_PIN_A;
    const gpio_num_t b = NAVI_USS_PIN_B;
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (uss_try(a, b)) {
            s_trig = a;
            s_echo = b;
        } else if (uss_try(b, a)) {
            s_trig = b;
            s_echo = a;
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        gpio_config_t t = {};
        t.pin_bit_mask = (1ULL << s_trig);
        t.mode = GPIO_MODE_OUTPUT;
        t.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&t);
        gpio_set_level(s_trig, 0);
        ESP_LOGI(TAG, "ultrasonic found, trig GPIO%d echo GPIO%d",
                 s_trig, s_echo);
        return true;
    }
    s_trig = GPIO_NUM_NC;
    s_echo = GPIO_NUM_NC;
    return false;
}

// One measurement. Busy waits, which is fine on a low priority sensor task.
static int32_t uss_measure() {
    gpio_set_level(s_trig, 0);
    esp_rom_delay_us(4);
    gpio_set_level(s_trig, 1);
    esp_rom_delay_us(10);            // 10 us trigger pulse the part needs
    gpio_set_level(s_trig, 0);

    if (!wait_level(s_echo, 1, ECHO_TIMEOUT_US)) {
        return USS_NO_READING;       // echo never started
    }
    const int64_t echo_start = esp_timer_get_time();
    if (!wait_level(s_echo, 0, ECHO_TIMEOUT_US)) {
        // Nothing in range. Let the long no target pulse finish so the next
        // trigger is not ignored.
        wait_level(s_echo, 0, ECHO_IDLE_WAIT_US);
        return USS_NO_READING;
    }
    return echo_us_to_mm(esp_timer_get_time() - echo_start);
}

static volatile bool s_uss_diag_requested = false;

void sensors_request_uss_diag() {
    s_uss_diag_requested = true;
}

// Idle level with each pull. Following the pull means nothing drives the pin.
static const char* uss_pin_idle(gpio_num_t pin) {
    gpio_config_t c = {};
    c.pin_bit_mask = (1ULL << pin);
    c.mode = GPIO_MODE_INPUT;
    c.intr_type = GPIO_INTR_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&c);
    esp_rom_delay_us(200);
    const int with_down = gpio_get_level(pin);
    c.pull_down_en = GPIO_PULLDOWN_DISABLE;
    c.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&c);
    esp_rom_delay_us(200);
    const int with_up = gpio_get_level(pin);
    uss_set_input(pin);
    if (with_down == 0 && with_up == 1) return "floating (nothing driving it)";
    if (with_down == 1 && with_up == 1) return "driven HIGH";
    if (with_down == 0 && with_up == 0) return "driven LOW";
    return "odd (high with pull-down, low with pull-up)";
}

// Pulse trig, then watch echo for 100 ms.
static void uss_diag_direction(gpio_num_t trig, gpio_num_t echo) {
    uss_set_input(trig);
    uss_set_input(echo);
    esp_rom_delay_us(500);
    const int echo_before = gpio_get_level(echo);
    gpio_set_level(trig, 0);
    gpio_set_direction(trig, GPIO_MODE_OUTPUT);
    esp_rom_delay_us(4);
    gpio_set_level(trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig, 0);

    const int64_t start = esp_timer_get_time();
    int64_t rise = -1, high_us = 0, last_change = start;
    int level = gpio_get_level(echo), edges = 0;
    while (esp_timer_get_time() - start < 100000) {
        const int now_level = gpio_get_level(echo);
        if (now_level != level) {
            const int64_t t = esp_timer_get_time();
            if (now_level == 1 && rise < 0) rise = t - start;
            if (level == 1) high_us += t - last_change;
            last_change = t;
            level = now_level;
            ++edges;
        }
    }
    if (level == 1) high_us += esp_timer_get_time() - last_change;
    uss_set_input(trig);
    ESP_LOGI(TAG, "ussdiag: trig GPIO%d -> echo GPIO%d: echo before=%d, edges=%d, "
                  "first rise %s%lld us, high for %lld us total",
             trig, echo, echo_before, edges, rise < 0 ? "never " : "at ",
             (long long)(rise < 0 ? 0 : rise), (long long)high_us);
}

static void uss_diag() {
    const gpio_num_t a = NAVI_USS_PIN_A;
    const gpio_num_t b = NAVI_USS_PIN_B;
    ESP_LOGI(TAG, "ussdiag: GPIO%d idle %s", a, uss_pin_idle(a));
    ESP_LOGI(TAG, "ussdiag: GPIO%d idle %s", b, uss_pin_idle(b));
    uss_diag_direction(a, b);
    vTaskDelay(pdMS_TO_TICKS(60));
    uss_diag_direction(b, a);
    ESP_LOGI(TAG, "ussdiag: a working sensor shows edges=2 and a rise within "
                  "about 1 ms on one of the two lines");
    // Leave the pins as normal polling expects them.
    if (s_trig != GPIO_NUM_NC) {
        gpio_set_direction(s_trig, GPIO_MODE_OUTPUT);
        gpio_set_level(s_trig, 0);
    }
}

static void uss_poll(SensorState& out) {
    if (s_uss_diag_requested) {
        s_uss_diag_requested = false;
        uss_diag();
    }
    if (s_trig == GPIO_NUM_NC) {
        // Not found yet. Try again about every 2 s so a sensor plugged in
        // after boot is picked up.
        if (++s_uss_retry >= 40) {
            s_uss_retry = 0;
            uss_detect();
        }
    }
    out.uss_ok = s_trig != GPIO_NUM_NC;
    out.uss_mm = out.uss_ok ? uss_measure() : USS_NO_READING;

    // bumper_update takes two sensors; the PCB has one.
    out.bumper = bumper_update(s_bumper, out.uss_mm, USS_NO_READING,
                               esp_timer_get_time(), BUMPER_STOP_MM,
                               BUMPER_RELEASE_MM, BUMPER_CLEAR_MS);
}

// Public interface

bool sensors_init() {
    if (!uss_detect()) {
        // Without it the bumper is off, so say so loudly.
        ESP_LOGE(TAG, "no ultrasonic answering on GPIO%d/GPIO%d. Bumper is "
                      "OFF until one is found. Check the U1 cable and power.",
                 NAVI_USS_PIN_A, NAVI_USS_PIN_B);
    }
    s_imu_ok = imu_init();
    if (!s_imu_ok) {
        sensors_i2c_scan();
    }
    return s_imu_ok;
}

void sensors_i2c_scan() {
    if (s_bus == nullptr) {
        ESP_LOGE(TAG, "i2c scan: bus never started");
        return;
    }
    if (s_imu_ok) {
        ESP_LOGI(TAG, "i2c scan: skipped, IMU is running at 0x%02X", s_imu_addr);
        return;
    }
    if (s_imu) {
        i2c_master_bus_rm_device(s_imu);
        s_imu = nullptr;
    }

    // Idle levels. With only the internal pull ups both should read 1. A 0
    // means something holds the line low.
    const int sda = gpio_get_level(NAVI_I2C_SDA_PIN);
    const int scl = gpio_get_level(NAVI_I2C_SCL_PIN);
    ESP_LOGI(TAG, "i2c scan: idle SDA(GPIO%d)=%d SCL(GPIO%d)=%d%s",
             NAVI_I2C_SDA_PIN, sda, NAVI_I2C_SCL_PIN, scl,
             (sda && scl) ? "" : "  <-- line held low");
    if (!sda || !scl) {
        i2c_master_bus_reset(s_bus);
        ESP_LOGI(TAG, "i2c scan: after bus reset SDA=%d SCL=%d",
                 gpio_get_level(NAVI_I2C_SDA_PIN), gpio_get_level(NAVI_I2C_SCL_PIN));
    }

    char found[96] = "";
    int n_found = 0, n_timeout = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        const esp_err_t err = i2c_master_probe(s_bus, addr, 20);
        if (err == ESP_OK) {
            const size_t used = strlen(found);
            snprintf(found + used, sizeof(found) - used, " 0x%02X", addr);
            ++n_found;
        } else if (err == ESP_ERR_TIMEOUT) {
            ++n_timeout;
        }
    }
    ESP_LOGI(TAG, "i2c scan: %d device(s) ACK:%s", n_found,
             n_found ? found : " none");
    if (n_timeout) {
        ESP_LOGW(TAG, "i2c scan: %d address(es) timed out, bus may be stuck",
                 n_timeout);
    }

    // WHO_AM_I at a gentle 100 kHz, in case 400 kHz is too fast for the weak
    // internal pull ups.
    const uint8_t addrs[2] = {ICM_ADDR_AD0_LOW, ICM_ADDR_AD0_HIGH};
    for (uint8_t addr : addrs) {
        i2c_device_config_t cfg = {};
        cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        cfg.device_address = addr;
        cfg.scl_speed_hz = 100000;
        i2c_master_dev_handle_t dev = nullptr;
        if (i2c_master_bus_add_device(s_bus, &cfg, &dev) != ESP_OK) {
            continue;
        }
        const uint8_t bank0[2] = {REG_BANK_SEL, 0};
        i2c_master_transmit(dev, bank0, sizeof(bank0), 50);
        uint8_t reg = REG_WHO_AM_I, id = 0;
        const esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, &id, 1, 50);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "i2c scan: 0x%02X WHO_AM_I = 0x%02X (ICM-20948 is 0xEA)",
                     addr, id);
        } else {
            ESP_LOGI(TAG, "i2c scan: 0x%02X WHO_AM_I read failed (%s)",
                     addr, esp_err_to_name(err));
        }
        i2c_master_bus_rm_device(dev);
    }
}

void sensors_poll(SensorState& out) {
    imu_poll(out);
    uss_poll(out);
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
