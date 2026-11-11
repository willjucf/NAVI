// Wireless links to the ESP32 for bench work: logs and commands without a
// cable, and firmware updates over WiFi.
//
// Two extra ways in, both speaking the same <L,R,F> packets as the Pi link:
//   WiFi: joins the hotspot below, TCP server on NAVI_WIFI_TCP_PORT
//   BLE:  Nordic UART Service, advertised as NAVI_BLE_NAME
//   OTA:  new firmware over WiFi (NAVI_OTA_TCP_PORT) or BLE (its own update
//         service), no cable or buttons. tools/navi_ota.py sends it.
//
// Text commands, one per line, outside any <...> packet, from the UART, WiFi
// or BLE:
//   i2cscan   rerun the I2C bus diagnostic (sensors_i2c_scan)
//   ussdiag   ultrasonic header diagnostic (sensors_request_uss_diag)
//   beep      1 kHz then 2 kHz on the aux board speaker (speaker_tone)
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

// Feeds one byte from the UART to the text command reader, so the commands
// above also work over the CP2102. Call from the UART receive task only.
void wireless_uart_text(char c);

}  // namespace navi
