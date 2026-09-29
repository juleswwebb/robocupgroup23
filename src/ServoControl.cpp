#include "ServoControl.h"
#include "sensor_config.h"

#include <Arduino.h>
#include <Servo.h>

static Servo testServo;
static int currentMicroseconds = SERVO_TEST_NEUTRAL_US;
static int currentAngleDegrees = 90;
static unsigned long lastCommandMs = 0;
static bool initialized = false;
static bool positionMode = false;

void servo_control_init() {
    positionMode = false;
    currentMicroseconds = SERVO_TEST_NEUTRAL_US;
    currentAngleDegrees = 90;
    testServo.attach(SERVO_TEST_PIN);
    testServo.writeMicroseconds(currentMicroseconds);
    lastCommandMs = millis();
    initialized = true;
}

int servo_control_get_pin() {
    return SERVO_TEST_PIN;
}

void servo_control_set_microseconds(int microseconds) {
    positionMode = false;
    currentMicroseconds = constrain(microseconds,
                                    SERVO_TEST_MIN_US,
                                    SERVO_TEST_MAX_US);
    lastCommandMs = millis();
    if (initialized) {
        testServo.writeMicroseconds(currentMicroseconds);
    }
}

void servo_control_set_speed(int percent) {
    percent = constrain(percent, -100, 100);
    servo_control_set_microseconds(
        SERVO_TEST_NEUTRAL_US + percent * 5);
}

void servo_control_set_angle(int degrees) {
    degrees = constrain(degrees, 0, 180);
    positionMode = true;
    currentAngleDegrees = degrees;
    // Servo.write(degrees) requests a positional target. Servo-style PWM
    // drivers for continuous-rotation motors interpret the same signal as
    // speed instead, so the app labels this mode as positional-only.
    currentMicroseconds = map(degrees, 0, 180, 544, 2400);
    lastCommandMs = millis();
    if (initialized) {
        testServo.write(degrees);
    }
}

void servo_control_stop() {
    // A positional servo should hold its requested position; neutral pulses
    // are only meaningful for the continuous-rotation / ESC pulse test mode.
    if (positionMode) {
        return;
    }
    currentMicroseconds = SERVO_TEST_NEUTRAL_US;
    lastCommandMs = millis();
    if (initialized) {
        testServo.writeMicroseconds(currentMicroseconds);
    }
}

void servo_control_update() {
    if (initialized && !positionMode &&
        currentMicroseconds != SERVO_TEST_NEUTRAL_US &&
        millis() - lastCommandMs > SERVO_TEST_COMMAND_TIMEOUT_MS) {
        servo_control_stop();
    }
}

bool servo_control_is_active() {
    return !positionMode && currentMicroseconds != SERVO_TEST_NEUTRAL_US;
}

int servo_control_get_microseconds() {
    return currentMicroseconds;
}

int servo_control_get_angle() {
    return currentAngleDegrees;
}

bool servo_control_is_position_mode() {
    return positionMode;
}

void servo_control_print() {
    Serial.print("servo_test: D");
    Serial.print(SERVO_TEST_PIN);
    if (positionMode) {
        Serial.print(" angle=");
        Serial.print(currentAngleDegrees);
        Serial.println(" deg");
    } else {
        Serial.print(" ");
        Serial.print(currentMicroseconds);
        Serial.println(" us");
    }
}
