// Protocol echo tool.
//
// Reads raw bytes on stdin, feeds them through the real firmware PacketParser
// one byte at a time exactly as the UART task does, and prints one line per
// decoded frame:
//
//     OK <left> <right> <flag>
//     BAD <error name>
//
// This exists so the Pi side and the ESP32 side can be checked against each
// other rather than each being checked against my description of the format.
// selftest.py generates packets with the Python formatter and pipes them in
// here, so if the two implementations ever disagree about framing, spacing,
// sign handling or clamping, the disagreement is caught on a laptop instead of
// on a chair with motors attached.
//
// Build:
//   g++ -std=c++17 -I../main protocol_echo.cpp ../main/navi_logic.cpp -o protocol_echo

#include <cstdio>

#include "navi_logic.h"

using namespace navi;

static const char* error_name(ParseError e) {
    switch (e) {
        case ParseError::PACKET_CORRUPTION_ERROR: return "PACKET_CORRUPTION_ERROR";
        case ParseError::PAYLOAD_TOO_LONG:        return "PAYLOAD_TOO_LONG";
        default:                                  return "NONE";
    }
}

int main() {
    PacketParser parser;
    DriveCommand cmd;
    int c;

    while ((c = std::getchar()) != EOF) {
        if (parser.feed(static_cast<char>(c), cmd)) {
            std::printf("OK %d %d %d\n", cmd.left, cmd.right,
                        cmd.safety_flag ? 1 : 0);
        } else if (parser.last_error() != ParseError::NONE) {
            std::printf("BAD %s\n", error_name(parser.last_error()));
        }
    }
    std::fflush(stdout);
    return 0;
}
