#include "ServoControl.h"
#include "sensor_config.h"
#include <Arduino.h>
#include <Servo.h>

static Servo servoD28;
static Servo servoD29;

#define SERVO_STOP_US 1500
#define SERVO_MIN_US   500
#define SERVO_MAX_US  2500

static int currentMicroseconds = SERVO_STOP_US;

void servo_control_init() {
    servoD28.attach(SERVO_D28_PIN);
    servoD29.attach(SERVO_D29_PIN);
    servo_control_set_microseconds(SERVO_STOP_US);
}

void servo_control_set_microseconds(int microseconds) {
    currentMicroseconds = constrain(microseconds, SERVO_MIN_US, SERVO_MAX_US);
    servoD28.writeMicroseconds(currentMicroseconds);
    servoD29.writeMicroseconds(currentMicroseconds);
}

void servo_control_set_speed(int percent) {
    percent = constrain(percent, -100, 100);
    servo_control_set_microseconds(1500 + percent * 5); // -100..100 -> 1000..2000
}

void servo_control_set_angle(int degrees) {
    degrees = constrain(degrees, 0, 180);
    servoD28.write(degrees);
    servoD29.write(degrees);
    currentMicroseconds = servoD28.readMicroseconds();
}

void servo_control_print() {
    Serial.print("servo_control: ");
    Serial.print(currentMicroseconds);
    Serial.println(" us");
}
