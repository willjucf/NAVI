// E-stop and safe state. The button is push to stop, release to run.

#pragma once

#include <cstdint>

#include "navi_logic.h"   // HaltReason lives there so it can be unit tested

namespace navi {

// Set up the e stop interrupt and put relay, LED and buzzer in a known state.
// Call once at startup before the control loop.
void safety_init();

// True while in ESTOP. Cleared by releasing the button (estop_poll_release).
bool estop_active();

// The latched reason. Stays set while halted.
HaltReason halt_reason();

// Only the button latches. Link loss, bumper and tilt clear on their own.

// Leave the safe state. Returns false while the button is still pressed.
bool safety_reset();

// Called every pass: clears the stop once the button has read released for ESTOP_RELEASE_US.
// Doesn't close the relay; safety_arm() does once every gate passes.
void estop_poll_release();

// Square-wave tone on the buzzer pin for ms. Blocks; call from its own task.
void speaker_tone(uint32_t hz, uint32_t ms);

// Close the relays. They boot open.
void safety_arm();

// Zero the motor outputs. Task context only; the ISR uses motors_cut_from_isr().
void motors_halt();

// ISR-safe stop: opens the relays and lights the indicators with GPIO writes only.
void motors_cut_from_isr();

// Drive the buzzer and LED to follow a halt. Task context. The
// ISR writes the same pins directly.
void set_indicators(bool on);

}  // namespace navi
