#include "ServoControl.h"
#include <Arduino.h>

#define SERVO_STOP_US 1500
#define SERVO_MIN_US   500
#define SERVO_MAX_US  2500

static int currentMicroseconds = SERVO_STOP_US;

void servo_control_init() {
    // Retired: D28/D29 now belong exclusively to the drum controller.
    currentMicroseconds = SERVO_STOP_US;
}

void servo_control_set_microseconds(int microseconds) {
    currentMicroseconds = constrain(microseconds, SERVO_MIN_US, SERVO_MAX_US);
}

void servo_control_set_speed(int percent) {
    percent = constrain(percent, -100, 100);
    servo_control_set_microseconds(1500 + percent * 5); // -100..100 -> 1000..2000
}

void servo_control_set_angle(int degrees) {
    degrees = constrain(degrees, 0, 180);
    currentMicroseconds = map(degrees, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
}

int servo_control_get_microseconds() { return currentMicroseconds; }

void servo_control_print() {
    Serial.print("servo_control: ");
    Serial.print(currentMicroseconds);
    Serial.println(" us");
}
