// Speaker on the aux board. Console commands:
//   tones        test tones, a sweep and beeps
//   say <clip>   estop, obstacle, red, tilt, link, resume, voice or all
// One sound at a time on its own task.

#pragma once

namespace navi {

// Start the sound task. Call once from app_main.
void sound_init();

// Queue a sound. False, with a log line saying why, if one is already playing
// or the clip name is unknown.
bool sound_tones();
bool sound_say(const char* name);

// True while a sound plays. beep checks it so the two don't share the pin.
bool sound_busy();

}  // namespace navi
