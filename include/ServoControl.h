//************************************
//         ServoControl.h
//************************************
//
// Manual control for the single servo on the "SERIAL7" connector (labelled
// that only because D28/D29 double as Serial7 RX/TX - here they're just
// plain PWM outputs). We don't yet know which of D28/D29 is the actual
// signal wire, so both are driven identically - harmless either way, since
// a GPIO toggling into an unconnected wire does nothing.
//
// Confirmed to be a continuous-rotation servo (writeMicroseconds at the
// 1000/2000us extremes = full speed reverse/forward, 1500us = stop), so
// setSpeed()/setMicroseconds() are the primary controls; setAngle() is
// kept too in case a positional servo ends up on this connector later.

#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

// Attaches both candidate pins and holds neutral (stopped). Call once from setup().
void servo_control_init();

// Direct pulse width, clamped to a safe 500-2500us range. 1500 = stop
// (for a continuous-rotation servo) or roughly centre (for a positional one).
void servo_control_set_microseconds(int microseconds);

// -100..100, mapped linearly onto 1000-2000us (0 = stop/centre).
void servo_control_set_speed(int percent);

// 0..180 degrees, for a positional servo (uses the angle API, not raw pulses).
void servo_control_set_angle(int degrees);

// Print the current commanded pulse width to Serial. For debugging.
void servo_control_print();

// Currently commanded pulse width, us.
int servo_control_get_microseconds();

#endif /* SERVO_CONTROL_H_ */
