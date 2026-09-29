// Hextronik HX12K positional servo on Teensy D20; raw pulse mode is diagnostic.
#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

void servo_control_init();
int servo_control_get_pin();
void servo_control_set_microseconds(int microseconds);
void servo_control_set_speed(int percent);
void servo_control_set_angle(int degrees);
void servo_control_stop();
void servo_control_update();
bool servo_control_is_active();
int servo_control_get_microseconds();
int servo_control_get_angle();
bool servo_control_is_position_mode();
void servo_control_print();

#endif /* SERVO_CONTROL_H_ */
