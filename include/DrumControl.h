#ifndef DRUM_CONTROL_H_
#define DRUM_CONTROL_H_

// Two independent servo-style drum/roller driver channels. Neutral at boot,
// hard speed cap and 300 ms lost-command timeout. Do not use as a UART.
void drum_control_init();
void drum_control_update();
void drum_control_set_percent(int left, int right);
void drum_control_stop();
int drum_control_left_percent();
int drum_control_right_percent();
int drum_control_left_us();
int drum_control_right_us();
bool drum_control_is_active();

#endif
