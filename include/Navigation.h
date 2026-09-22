#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <stdint.h>

// Conservative autonomous motion built on Group 23's existing drive, IMU,
// encoders and 8x8 TOF interfaces. Navigation is off at boot and can only be
// enabled by the debug protocol while Debug Mode is armed.
void navigation_init();
void navigation_update();
bool navigation_set_enabled(bool enabled);
void navigation_stop(const char* reason = nullptr);

bool navigation_is_active();
const char* navigation_get_state_name();
const char* navigation_get_stop_reason();
uint16_t navigation_get_front_mm();
uint16_t navigation_get_left_mm();
uint16_t navigation_get_right_mm();
float navigation_get_target_heading();

void navigation_set_speed_percent(int value);
void navigation_set_turn_percent(int value);
void navigation_set_front_stop_mm(int value);
int navigation_get_speed_percent();
int navigation_get_turn_percent();
int navigation_get_front_stop_mm();

#endif
