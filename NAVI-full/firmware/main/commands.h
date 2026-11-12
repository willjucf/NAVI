// UART text commands, outside any <...> packet: i2cscan, ussdiag, beep.

#pragma once

namespace navi {

// Feeds one byte from the UART. Call from the UART receive task only.
void commands_uart_text(char c);

}  // namespace navi
