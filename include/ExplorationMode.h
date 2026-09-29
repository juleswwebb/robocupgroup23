#ifndef EXPLORATION_MODE_H_
#define EXPLORATION_MODE_H_

#include <stdint.h>

// Arena-free, robot-side autonomous exploration. The GUI starts/stops a run
// and must refresh its lease while connected; loss of that lease stops both
// drive and drum outputs.
void exploration_mode_init();
bool exploration_mode_start();
void exploration_mode_stop(const char* reason = nullptr);
void exploration_mode_keepalive();
void exploration_mode_update();

bool exploration_mode_is_active();
const char* exploration_mode_state();
const char* exploration_mode_reason();
uint16_t exploration_mode_front_mm();
uint16_t exploration_mode_left_mm();
uint16_t exploration_mode_right_mm();
uint16_t exploration_mode_left_top_mm();
uint16_t exploration_mode_left_bottom_mm();
uint16_t exploration_mode_right_top_mm();
uint16_t exploration_mode_right_bottom_mm();
uint16_t exploration_mode_weights_seen();
uint16_t exploration_mode_turns();
float exploration_mode_target_bearing_deg();

#endif
