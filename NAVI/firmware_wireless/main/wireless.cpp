// Temporary WiFi and BLE links. See wireless.h.
//
// Nothing here can drive a motor by itself. Valid packets go to the same sink
// as UART packets, and the control task applies every safety gate to them.
// A failure in this file logs and carries on, so the safety firmware still
// boots without a radio.

#include "wireless.h"
#include "sensors.h"
#include "safety.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace navi {

static const char* TAG = "wireless";

static CommandSink s_sink = nullptr;

// Log mirror
//
// The hook runs inside whatever task logged, so it only copies the line into a
// ring buffer. A separate task sends it out, and a slow phone never stalls the
// control loop. Lines are dropped, not queued, when the buffer is full.

static RingbufHandle_t s_log_rb = nullptr;
static vprintf_like_t s_uart_vprintf = nullptr;
static TaskHandle_t s_mirror_task = nullptr;

static volatile int s_tcp_fd = -1;
static volatile uint16_t s_ble_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_ble_subscribed = false;
static volatile uint32_t s_ip = 0;

static bool any_client() {
    return s_tcp_fd >= 0 || s_ble_subscribed;
}

static int log_hook(const char* fmt, va_list args) {
    va_list copy;
    va_copy(copy, args);
    const int n = s_uart_vprintf(fmt, args);   // UART console unchanged
    // Never mirror what the mirror task itself logs. Sending a line can make
    // the BLE or TCP stack log, and mirroring that would loop forever.
    if (any_client() && xTaskGetCurrentTaskHandle() != s_mirror_task) {
        char line[200];
        int len = vsnprintf(line, sizeof(line), fmt, copy);
        if (len > 0) {
            if (len >= (int)sizeof(line)) {
                len = sizeof(line) - 1;
            }
            xRingbufferSend(s_log_rb, line, len, 0);
        }
    }
    va_end(copy);
    return n;
}

static void tcp_send(const char* data, size_t len) {
    const int fd = s_tcp_fd;
    if (fd >= 0) {
        send(fd, data, len, MSG_DONTWAIT);   // a full socket drops the line
    }
}

// Text commands
//
// Collects bytes outside <...> into a line. The packet parser already skips
// them, so both can be fed every byte. Commands run in their own task, since
// a scan on a stuck bus can take seconds and must not block a radio task.

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

// Firmware updates, shared by WiFi and BLE
//
// One update at a time. The image goes to the spare OTA slot and only becomes
// the boot image after esp_ota_end has checked it, so a failed or cut-off
// update leaves the running firmware untouched. The reboot at the end drops
// the relay like any reset, so the chair is left stopped.

static portMUX_TYPE s_ota_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_ota_busy = false;

static bool ota_lock() {
    bool got = false;
    taskENTER_CRITICAL(&s_ota_mux);
    if (!s_ota_busy) {
        s_ota_busy = true;
        got = true;
    }
    taskEXIT_CRITICAL(&s_ota_mux);
    return got;
}

static void ota_unlock() {
    s_ota_busy = false;
}

static void restart_task(void* arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

// Reboot a second from now, so the reply to the client gets out first.
static void schedule_restart() {
    xTaskCreate(restart_task, "restart", 2048, nullptr, 1, nullptr);
}

// BLE, Nordic UART Service

// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E and friends, little endian.
static const ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_RX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_TX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t s_tx_handle = 0;
static uint8_t s_own_addr_type = 0;
static PacketParser s_ble_parser;       // NimBLE host task only
static TextCommand s_ble_text;
static TextCommand s_uart_text;      // UART receive task only

void wireless_uart_text(char c) {
    s_uart_text.feed(c);
}

static void ble_send(const char* data, size_t len) {
    const uint16_t conn = s_ble_conn;
    // During an update the link is busy carrying the image.
    if (!s_ble_subscribed || conn == BLE_HS_CONN_HANDLE_NONE || s_ota_busy) {
        return;
    }
    size_t chunk = ble_att_mtu(conn);
    chunk = chunk > 3 ? chunk - 3 : 20;
    for (size_t off = 0; off < len; off += chunk) {
        const size_t n = (len - off < chunk) ? len - off : chunk;
        os_mbuf* om = ble_hs_mbuf_from_flat(data + off, n);
        if (om == nullptr || ble_gatts_notify_custom(conn, s_tx_handle, om) != 0) {
            return;                      // out of buffers, drop the rest
        }
    }
}

static int nus_access(uint16_t conn, uint16_t attr, ble_gatt_access_ctxt* ctxt,
                      void* arg) {
    (void)conn;
    (void)attr;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    uint8_t buf[128];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    DriveCommand decoded;
    for (uint16_t i = 0; i < len; ++i) {
        s_ble_text.feed(static_cast<char>(buf[i]));
        if (s_ble_parser.feed(static_cast<char>(buf[i]), decoded)) {
            s_sink(decoded, esp_timer_get_time());
        }
    }
    // Phone apps often send a line without a newline, so a write that ends
    // mid line is treated as a complete one.
    s_ble_text.feed('\n');
    return 0;
}

// BLE firmware update service
//
// Write "BEGIN <bytes>" to the control characteristic, the app .bin in chunks
// to the data characteristic, then "END" (or "ABORT"). Every write needs a
// response, so the client is paced and nothing is dropped. A failed write
// returns one of the OTA_ERR codes below. Disconnecting mid update aborts it.

// d0f20001-8a4e-4b5a-9c1e-4e4156494f54 and friends, little endian.
static const ble_uuid128_t OTA_SVC_UUID = BLE_UUID128_INIT(
    0x54, 0x4f, 0x49, 0x56, 0x41, 0x4e, 0x1e, 0x9c,
    0x5a, 0x4b, 0x4e, 0x8a, 0x01, 0x00, 0xf2, 0xd0);
static const ble_uuid128_t OTA_CTRL_UUID = BLE_UUID128_INIT(
    0x54, 0x4f, 0x49, 0x56, 0x41, 0x4e, 0x1e, 0x9c,
    0x5a, 0x4b, 0x4e, 0x8a, 0x02, 0x00, 0xf2, 0xd0);
static const ble_uuid128_t OTA_DATA_UUID = BLE_UUID128_INIT(
    0x54, 0x4f, 0x49, 0x56, 0x41, 0x4e, 0x1e, 0x9c,
    0x5a, 0x4b, 0x4e, 0x8a, 0x03, 0x00, 0xf2, 0xd0);

// ATT application error codes, so the client can say why a write failed.
static constexpr int OTA_ERR_BUSY = 0x80;    // another update is running
static constexpr int OTA_ERR_SIZE = 0x81;    // size is 0 or too big for the slot
static constexpr int OTA_ERR_BEGIN = 0x82;   // esp_ota_begin failed
static constexpr int OTA_ERR_WRITE = 0x83;   // flash write failed or too much data
static constexpr int OTA_ERR_IMAGE = 0x84;   // the image did not check out
static constexpr int OTA_ERR_STATE = 0x85;   // data or END with no update running

static uint16_t s_ota_ctrl_handle = 0;
static uint16_t s_ota_data_handle = 0;

// NimBLE host task only.
static bool s_ble_ota_active = false;
static esp_ota_handle_t s_ble_ota = 0;
static const esp_partition_t* s_ble_ota_part = nullptr;
static uint32_t s_ble_ota_size = 0;
static uint32_t s_ble_ota_got = 0;
static int s_ble_ota_pct = 0;

static void ble_ota_abort(const char* why) {
    if (!s_ble_ota_active) {
        return;
    }
    esp_ota_abort(s_ble_ota);
    s_ble_ota_active = false;
    ota_unlock();
    ESP_LOGE(TAG, "ota (ble): aborted, %s", why);
}

static int ble_ota_ctrl(const char* cmd) {
    if (strncmp(cmd, "BEGIN ", 6) == 0) {
        ble_ota_abort("restarted by client");
        if (!ota_lock()) {
            return OTA_ERR_BUSY;
        }
        const unsigned long size = strtoul(cmd + 6, nullptr, 10);
        const esp_partition_t* part = esp_ota_get_next_update_partition(nullptr);
        if (part == nullptr || size == 0 || size > part->size) {
            ota_unlock();
            return OTA_ERR_SIZE;
        }
        if (esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &s_ble_ota) != ESP_OK) {
            ota_unlock();
            return OTA_ERR_BEGIN;
        }
        s_ble_ota_part = part;
        s_ble_ota_size = size;
        s_ble_ota_got = 0;
        s_ble_ota_pct = 0;
        s_ble_ota_active = true;
        ESP_LOGW(TAG, "ota (ble): receiving %lu bytes into %s", size, part->label);
        return 0;
    }
    if (strcmp(cmd, "END") == 0) {
        if (!s_ble_ota_active || s_ble_ota_got != s_ble_ota_size) {
            ble_ota_abort("END before all the data");
            return OTA_ERR_STATE;
        }
        s_ble_ota_active = false;
        const esp_err_t err = esp_ota_end(s_ble_ota);      // checks the image
        if (err != ESP_OK || esp_ota_set_boot_partition(s_ble_ota_part) != ESP_OK) {
            ota_unlock();
            ESP_LOGE(TAG, "ota (ble): image rejected (%s)", esp_err_to_name(err));
            return OTA_ERR_IMAGE;
        }
        ESP_LOGW(TAG, "ota (ble): done, rebooting into %s", s_ble_ota_part->label);
        schedule_restart();
        return 0;
    }
    if (strcmp(cmd, "ABORT") == 0) {
        ble_ota_abort("by client");
        return 0;
    }
    return OTA_ERR_STATE;
}

static int ota_access(uint16_t conn, uint16_t attr, ble_gatt_access_ctxt* ctxt,
                      void* arg) {
    (void)conn;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    static uint8_t buf[520];             // one write at the 512 byte MTU
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &len) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    if (attr == s_ota_ctrl_handle) {
        while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
            --len;
        }
        buf[len] = 0;
        return ble_ota_ctrl(reinterpret_cast<const char*>(buf));
    }

    if (!s_ble_ota_active) {
        return OTA_ERR_STATE;
    }
    if (s_ble_ota_got + len > s_ble_ota_size ||
        esp_ota_write(s_ble_ota, buf, len) != ESP_OK) {
        ble_ota_abort("flash write failed");
        return OTA_ERR_WRITE;
    }
    s_ble_ota_got += len;
    const int pct = (int)((uint64_t)s_ble_ota_got * 100 / s_ble_ota_size);
    if (pct / 10 != s_ble_ota_pct / 10) {
        s_ble_ota_pct = pct;
        ESP_LOGI(TAG, "ota (ble): %d%%", pct);
    }
    return 0;
}

static const ble_gatt_chr_def OTA_CHRS[] = {
    {
        .uuid = &OTA_CTRL_UUID.u,
        .access_cb = ota_access,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &s_ota_ctrl_handle,
    },
    {
        .uuid = &OTA_DATA_UUID.u,
        .access_cb = ota_access,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &s_ota_data_handle,
    },
    {},
};

static const ble_gatt_chr_def NUS_CHRS[] = {
    {
        .uuid = &NUS_RX_UUID.u,
        .access_cb = nus_access,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &NUS_TX_UUID.u,
        .access_cb = nus_access,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_tx_handle,
    },
    {},
};

static const ble_gatt_svc_def BLE_SVCS[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &NUS_SVC_UUID.u,
        .characteristics = NUS_CHRS,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &OTA_SVC_UUID.u,
        .characteristics = OTA_CHRS,
    },
    {},
};

static int gap_event(ble_gap_event* ev, void* arg);

static void ble_advertise() {
    // Name in the advert, service UUID in the scan response. Both together
    // do not fit in 31 bytes.
    ble_hs_adv_fields adv = {};
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.name = reinterpret_cast<const uint8_t*>(NAVI_BLE_NAME);
    adv.name_len = strlen(NAVI_BLE_NAME);
    adv.name_is_complete = 1;
    ble_gap_adv_set_fields(&adv);

    ble_hs_adv_fields rsp = {};
    rsp.uuids128 = &NUS_SVC_UUID;
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    ble_gap_adv_params params = {};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    const int rc = ble_gap_adv_start(s_own_addr_type, nullptr, BLE_HS_FOREVER,
                                     &params, gap_event, nullptr);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble advertise failed, rc %d", rc);
    }
}

static int gap_event(ble_gap_event* ev, void* arg) {
    (void)arg;
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            s_ble_conn = ev->connect.conn_handle;
            s_ble_parser.reset();
            s_ble_text = TextCommand();
            ESP_LOGI(TAG, "ble client connected");
        } else {
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        s_ble_subscribed = false;
        s_ble_conn = BLE_HS_CONN_HANDLE_NONE;
        ble_ota_abort("connection lost");
        ESP_LOGI(TAG, "ble client disconnected");
        ble_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == s_tx_handle) {
            s_ble_subscribed = ev->subscribe.cur_notify;
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        break;
    default:
        break;
    }
    return 0;
}

static void ble_on_sync() {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ble_advertise();
    ESP_LOGI(TAG, "ble advertising as %s (UART and firmware update services)",
             NAVI_BLE_NAME);
}

static void ble_host_task(void* arg) {
    (void)arg;
    nimble_port_run();                   // returns only on nimble_port_stop
    nimble_port_freertos_deinit();
}

static void ble_start() {
    if (nimble_port_init() != ESP_OK) {
        ESP_LOGE(TAG, "ble init failed, continuing without it");
        return;
    }
    // NimBLE logs every notify at info level, which floods the console and
    // the mirror. Warnings and errors are enough.
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_gatts_count_cfg(BLE_SVCS) != 0 || ble_gatts_add_svcs(BLE_SVCS) != 0) {
        ESP_LOGE(TAG, "ble service setup failed");
        return;
    }
    ble_svc_gap_device_name_set(NAVI_BLE_NAME);
    nimble_port_freertos_init(ble_host_task);
}

// WiFi

static void wifi_event(void* arg, esp_event_base_t base, int32_t id, void* data) {
    (void)arg;
    static int retries = 0;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_ip = 0;
        // Keep retrying forever, but only log now and then so a hotspot that
        // is switched off does not flood the console.
        if (retries++ % 10 == 0) {
            const auto* d = static_cast<wifi_event_sta_disconnected_t*>(data);
            ESP_LOGW(TAG, "wifi not connected (reason %d), retrying", d->reason);
        }
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto* e = static_cast<ip_event_got_ip_t*>(data);
        s_ip = e->ip_info.ip.addr;
        retries = 0;
        ESP_LOGI(TAG, "wifi up, ip " IPSTR ", tcp port %d",
                 IP2STR(&e->ip_info.ip), NAVI_WIFI_TCP_PORT);
    }
}

static bool wifi_start() {
    esp_err_t err = nvs_flash_init();     // WiFi and BLE both keep state in NVS
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
        return false;
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&init) != ESP_OK) {
        ESP_LOGE(TAG, "wifi init failed, continuing without it");
        return false;
    }
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, nullptr);

    wifi_config_t cfg = {};
    strncpy(reinterpret_cast<char*>(cfg.sta.ssid), NAVI_WIFI_SSID,
            sizeof(cfg.sta.ssid));
    strncpy(reinterpret_cast<char*>(cfg.sta.password), NAVI_WIFI_PASSWORD,
            sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    // Modem sleep stays on (the default). WiFi and BLE share one radio and
    // the coexistence scheduler needs it.
    esp_wifi_start();
    ESP_LOGI(TAG, "wifi joining \"%s\"", NAVI_WIFI_SSID);
    return true;
}

static void tcp_server_task(void* arg) {
    (void)arg;
    const int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    const int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(NAVI_WIFI_TCP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
        listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "tcp server could not start");
        close(listen_fd);
        vTaskDelete(nullptr);
        return;
    }

    PacketParser parser;
    TextCommand text;
    DriveCommand decoded;
    char buf[64];

    while (true) {
        sockaddr_in peer = {};
        socklen_t peer_len = sizeof(peer);
        const int fd = accept(listen_fd, reinterpret_cast<sockaddr*>(&peer), &peer_len);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // Keepalive so a phone that walks out of range is dropped within
        // about 10 s instead of holding the slot forever.
        const int idle = 5, interval = 2, count = 3;
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        parser.reset();
        text = TextCommand();
        s_tcp_fd = fd;
        ESP_LOGI(TAG, "wifi client connected from %s", inet_ntoa(peer.sin_addr));

        while (true) {
            const int n = recv(fd, buf, sizeof(buf), 0);
            if (n <= 0) {
                break;
            }
            for (int i = 0; i < n; ++i) {
                text.feed(buf[i]);
                if (parser.feed(buf[i], decoded)) {
                    s_sink(decoded, esp_timer_get_time());
                }
            }
        }

        s_tcp_fd = -1;
        close(fd);
        ESP_LOGI(TAG, "wifi client disconnected");
    }
}

// Firmware update over WiFi
//
// Protocol: the client sends the image length as 4 bytes little endian, then
// the raw app .bin. The ESP replies "OK rebooting", or "ERR <reason>" and
// keeps running the old firmware.

static void ota_reply(int fd, const char* msg) {
    send(fd, msg, strlen(msg), 0);
    send(fd, "\n", 1, 0);
}

static void ota_receive(int fd) {
    if (!ota_lock()) {
        ota_reply(fd, "ERR another update is running");
        return;
    }
    uint32_t size = 0;
    if (recv(fd, &size, sizeof(size), MSG_WAITALL) != (int)sizeof(size)) {
        ota_reply(fd, "ERR no length");
        ota_unlock();
        return;
    }
    const esp_partition_t* part = esp_ota_get_next_update_partition(nullptr);
    if (part == nullptr) {
        ota_reply(fd, "ERR no OTA partition");
        ota_unlock();
        return;
    }
    if (size == 0 || size > part->size) {
        ota_reply(fd, "ERR image size");
        ota_unlock();
        return;
    }
    ESP_LOGW(TAG, "ota: receiving %lu bytes into %s", (unsigned long)size, part->label);

    esp_ota_handle_t ota = 0;
    if (esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        ota_reply(fd, "ERR ota begin");
        ota_unlock();
        return;
    }
    static char buf[1024];
    uint32_t got = 0;
    int last_pct = -1;
    while (got < size) {
        const int n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) {
            esp_ota_abort(ota);
            ESP_LOGE(TAG, "ota: connection lost at %lu bytes", (unsigned long)got);
            ota_unlock();
            return;
        }
        if (esp_ota_write(ota, buf, n) != ESP_OK) {
            esp_ota_abort(ota);
            ota_reply(fd, "ERR flash write");
            ota_unlock();
            return;
        }
        got += n;
        const int pct = (int)((uint64_t)got * 100 / size);
        if (pct / 25 != last_pct / 25) {
            last_pct = pct;
            ESP_LOGI(TAG, "ota: %d%%", pct);
        }
    }
    const esp_err_t err = esp_ota_end(ota);      // checks the image
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ota: image rejected (%s)", esp_err_to_name(err));
        ota_reply(fd, "ERR image invalid");
        ota_unlock();
        return;
    }
    if (esp_ota_set_boot_partition(part) != ESP_OK) {
        ota_reply(fd, "ERR set boot partition");
        ota_unlock();
        return;
    }
    ESP_LOGW(TAG, "ota: done, rebooting into %s", part->label);
    ota_reply(fd, "OK rebooting");
    schedule_restart();
}

static void ota_server_task(void* arg) {
    (void)arg;
    const int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    const int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(NAVI_OTA_TCP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
        listen(listen_fd, 1) != 0) {
        ESP_LOGE(TAG, "ota server could not start");
        close(listen_fd);
        vTaskDelete(nullptr);
        return;
    }
    while (true) {
        const int fd = accept(listen_fd, nullptr, nullptr);
        if (fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // A sender that stalls for 10 s is given up on.
        timeval tv = {.tv_sec = 10, .tv_usec = 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        ota_receive(fd);
        close(fd);
    }
}

// Sends mirrored log lines, and every 10 s says where to connect.
static void log_sender_task(void* arg) {
    (void)arg;
    int64_t last_hint_us = 0;
    while (true) {
        size_t size = 0;
        char* data = static_cast<char*>(
            xRingbufferReceiveUpTo(s_log_rb, &size, pdMS_TO_TICKS(500), 240));
        if (data != nullptr) {
            tcp_send(data, size);
            ble_send(data, size);
            vRingbufferReturnItem(s_log_rb, data);
        }

        const int64_t now = esp_timer_get_time();
        if (now - last_hint_us > 10000000) {
            last_hint_us = now;
            esp_ip4_addr_t ip = {.addr = s_ip};
            ESP_LOGI(TAG, "wifi %s " IPSTR ":%d (%s), ble %s (%s)",
                     s_ip ? "at" : "down,", IP2STR(&ip), NAVI_WIFI_TCP_PORT,
                     s_tcp_fd >= 0 ? "client" : "no client", NAVI_BLE_NAME,
                     s_ble_conn != BLE_HS_CONN_HANDLE_NONE ? "connected" : "advertising");
        }
    }
}

void wireless_start(CommandSink sink) {
    s_sink = sink;

    s_log_rb = xRingbufferCreate(4096, RINGBUF_TYPE_BYTEBUF);
    if (s_log_rb != nullptr) {
        s_uart_vprintf = esp_log_set_vprintf(log_hook);
        xTaskCreate(log_sender_task, "log_mirror", 4096, nullptr, 4, &s_mirror_task);
    }

    if (wifi_start()) {
        xTaskCreate(tcp_server_task, "tcp_server", 4096, nullptr, 5, nullptr);
        xTaskCreate(ota_server_task, "ota_server", 4096, nullptr, 5, nullptr);
    }
    ble_start();
}

}  // namespace navi
