// PWM control for the two main drive ESCs / continuous-rotation servos.
// The outputs are intentionally conservative: neutral at boot, a firmware
// watchdog, and a hard percentage limit.  This module is independent of the
// old placeholder motors.cpp module.

#ifndef DRIVE_CONTROL_H_
#define DRIVE_CONTROL_H_

#include <stdint.h>

void drive_control_init();
void drive_control_update();

// Requested values are -100..100. They are constrained by the configured
// maximum percentage before being sent to the two ESC signal pins.
void drive_control_set_percent(int left_percent, int right_percent);
void drive_control_stop();

// The configured test limit is itself hard-clamped to the safe absolute cap.
void drive_control_set_max_percent(int max_percent);
int drive_control_get_max_percent();

int drive_control_get_left_percent();
int drive_control_get_right_percent();
int drive_control_get_left_microseconds();
int drive_control_get_right_microseconds();
bool drive_control_is_active();

#endif /* DRIVE_CONTROL_H_ */
