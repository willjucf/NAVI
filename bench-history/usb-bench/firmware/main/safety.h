// NAVI emergency stop and safe state handling.
//
// Report 6.6 puts the physical emergency stop first among the safety gates,
// ahead of every other branch, and report 6.15 makes ESTOP a state that can be
// entered from anywhere and left only by an explicit reset. This module is
// where both of those live on the ESP32.

#pragma once

#include <cstdint>

#include "navi_logic.h"   // HaltReason lives there so it can be unit tested

namespace navi {

// Install the e stop interrupt and put the relay, LED and buzzer into a known
// state. Call once at startup before the control loop begins.
void safety_init();

// True while the system is latched in ESTOP. Cleared only by safety_reset().
bool estop_active();

// The latched reason. Stays set while the halt is active.
HaltReason halt_reason();

// NOTE on what latches and what does not.
//
// Only the button latches. It is the one condition a person deliberately
// asserted, so report 6.15 requires a deliberate action to clear it.
//
// Signal loss, the ultrasonic bumper and a tilt reading are all re evaluated
// from scratch on every control pass by evaluate_safety(), so they clear
// themselves when the condition clears. That is the behaviour you want: a
// chair should not need a manual reset because someone walked past the
// ultrasonic a moment ago, and it should resume when the Pi starts talking
// again. There is no safety_trigger() for that reason.

// Leave the safe state. Deliberately explicit: the system does not resume just
// because the button was released or the hazard left the camera view. Returns
// false and stays halted if the e stop pin is still held low.
//
// Report 6.15 requires that leaving ESTOP lands in MANUAL and never straight
// back in AUTO. The Pi owns the mode, so that rule is enforced there; this
// function only releases the hardware latch.
bool safety_reset();

// Close the motor cutoff relay, making the chair physically able to drive.
//
// The system boots with the relay open and stays that way until this is called,
// so nothing can move before the control loop is actually running. That is the
// initialization ordering report 6.4 describes.
void safety_arm();

// Drive the motor outputs to zero immediately, without ramping.
// A safety stop that eases in is not a safety stop.
//
// TASK CONTEXT ONLY. This touches the LEDC driver, which is not safe to call
// from an interrupt. Use motors_cut_from_isr() there instead.
void motors_halt();

// The interrupt safe half of a stop.
//
// Opens the cutoff relay and lights the indicators using only GPIO writes,
// which are in IRAM when CONFIG_GPIO_CTRL_FUNC_IN_IRAM is set, so this is safe
// to call with the flash cache disabled. Opening the relay is what actually
// removes power from the motors, so the electrical stop is immediate. The PWM
// registers are then zeroed by the control loop on its next pass, within one
// 10 ms period, which is two orders of magnitude inside the 500 ms budget in
// Table 2.4.1.
void motors_cut_from_isr();

// Indicator control, so the buzzer and LED can follow a halt (report 3.9).
// Task context. The ISR writes the same pins directly.
void set_indicators(bool on);

}  // namespace navi
