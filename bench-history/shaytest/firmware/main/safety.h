// NAVI emergency stop and safe state.
//
// Report 6.6 puts the physical e stop ahead of every other safety gate, and
// report 6.15 makes ESTOP enterable from anywhere but only left by an explicit
// reset. Both live here on the ESP32.

#pragma once

#include <cstdint>

#include "navi_logic.h"   // HaltReason lives there so it can be unit tested

namespace navi {

// Set up the e stop interrupt and put relay, LED and buzzer in a known state.
// Call once at startup before the control loop.
void safety_init();

// True while latched in ESTOP. Only safety_reset() clears it.
bool estop_active();

// The latched reason. Stays set while halted.
HaltReason halt_reason();

// What latches and what does not.
//
// Only the button latches. A person pressed it on purpose, so report 6.15 wants
// an on purpose action to clear it.
//
// Signal loss, the bumper and tilt are rechecked every pass by
// evaluate_safety() and clear on their own. A chair should not need a manual
// reset because someone walked past the ultrasonic, and it should resume when
// the Pi starts talking again. That is why there is no safety_trigger().

// Leave the safe state. Explicit on purpose so releasing the button or the
// hazard leaving view does not resume driving. Returns false and stays halted
// if the e stop pin is still low.
//
// Report 6.15 says leaving ESTOP must land in MANUAL, never AUTO. The Pi owns
// the mode so it enforces that. This only releases the hardware latch.
bool safety_reset();

// Close the motor cutoff relay so the chair can physically drive. The relay
// boots open and stays open until this runs, so nothing moves before the
// control loop is live (report 6.4 startup order).
void safety_arm();

// Zero the motor outputs right away with no ramp.
//
// Task context only. LEDC calls are not interrupt safe, so the ISR uses
// motors_cut_from_isr() instead.
void motors_halt();

// The interrupt safe half of a stop.
//
// Opens the relay and lights the indicators with GPIO writes only, which live
// in IRAM when CONFIG_GPIO_CTRL_FUNC_IN_IRAM is set, so it is safe with the
// flash cache off. Opening the relay cuts motor power, so the stop is instant.
// The control loop zeros the PWM on its next pass within 10 ms, far inside the
// 500 ms budget in Table 2.4.1.
void motors_cut_from_isr();

// Drive the buzzer and LED to follow a halt (report 3.9). Task context. The
// ISR writes the same pins directly.
void set_indicators(bool on);

}  // namespace navi
