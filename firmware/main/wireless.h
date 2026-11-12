// WiFi and BLE links for bench work. Both take the same <L,R,F> packets as the UART.
//   WiFi  TCP NAVI_WIFI_TCP_PORT, OTA on NAVI_OTA_TCP_PORT
//   BLE   Nordic UART Service as NAVI_BLE_NAME, plus an update service
// Text commands (UART, WiFi or BLE): i2cscan, ussdiag, beep.
// Every log line is mirrored to connected clients.

#pragma once

#include <cstdint>

#include "navi_logic.h"

// Hotspot name and password live in wifi_credentials.h, which is gitignored.
// Copy wifi_credentials.example.h to it and fill it in.
#include "wifi_credentials.h"
#define NAVI_WIFI_TCP_PORT  3333
#define NAVI_OTA_TCP_PORT   3334   // firmware updates, see tools/navi_ota.py

#define NAVI_BLE_NAME       "NAVI-ESP32"

namespace navi {

// Called with every valid packet from WiFi or BLE.
using CommandSink = void (*)(const DriveCommand& cmd, int64_t when_us);

// Starts the log mirror, WiFi and BLE. Call once from app_main.
void wireless_start(CommandSink sink);

// Feeds one byte from the UART to the text command reader, so the commands
// above also work over the CP2102. Call from the UART receive task only.
void wireless_uart_text(char c);

}  // namespace navi
