// Text commands over the UART, one per line, outside any <...> packet:
//   i2cscan   rerun the I2C bus diagnostic (sensors_i2c_scan)
//   ussdiag   ultrasonic header diagnostic (sensors_request_uss_diag)
//   beep      1 kHz then 2 kHz on the aux board speaker (speaker_tone)
//
// Same commands and replies as firmware_wireless. None of them moves a motor.

#pragma once

namespace navi {

// Feeds one byte from the UART. Call from the UART receive task only.
void commands_uart_text(char c);

}  // namespace navi
