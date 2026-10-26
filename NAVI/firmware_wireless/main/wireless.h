// Wireless links to the ESP32 for bench work: logs and commands without a
// cable, and firmware updates over WiFi.
//
// Two extra ways in, both speaking the same <L,R,F> packets as the Pi link:
//   WiFi: joins the hotspot below, TCP server on NAVI_WIFI_TCP_PORT
//   BLE:  Nordic UART Service, advertised as NAVI_BLE_NAME
//   OTA:  new firmware over WiFi on NAVI_OTA_TCP_PORT, no cable or buttons
//
// Text commands, one per line, outside any <...> packet:
//   i2cscan   rerun the I2C bus diagnostic (sensors_i2c_scan)
//
// Every log line is mirrored to whoever is connected, so either link also
// works as a remote serial monitor. Packets land in the same command slot as
// UART packets, so the 500 ms loss of signal watchdog and every safety gate
// apply unchanged. If the Pi and a phone both send, the newest packet wins.

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

}  // namespace navi
