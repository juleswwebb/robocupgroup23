#include "DrumControl.h"
#include "sensor_config.h"

#include <Arduino.h>
#include <Servo.h>

namespace {
Servo leftDrum;
Servo rightDrum;
int leftPercent = 0;
int rightPercent = 0;
int leftUs = 1500;
int rightUs = 1500;
unsigned long lastCommandMs = 0;
bool initialized = false;

int pulse_from_percent(int percent) {
    return constrain(1500 + (percent * 450) / 100, 1050, 1950);
}
}

void drum_control_init() {
    // Neutral pulses are sent before any nonzero command is possible.
    leftDrum.attach(DRUM_LEFT_PIN);
    rightDrum.attach(DRUM_RIGHT_PIN);
    initialized = true;
    drum_control_stop();
}

void drum_control_set_percent(int left, int right) {
    if (!initialized) return;
    leftPercent = constrain(left, -DRUM_TEST_MAX_PERCENT, DRUM_TEST_MAX_PERCENT);
    rightPercent = constrain(right, -DRUM_TEST_MAX_PERCENT, DRUM_TEST_MAX_PERCENT);
    leftUs = pulse_from_percent(leftPercent);
    rightUs = pulse_from_percent(rightPercent);
    leftDrum.writeMicroseconds(leftUs);
    rightDrum.writeMicroseconds(rightUs);
    lastCommandMs = millis();
}

void drum_control_stop() {
    leftPercent = rightPercent = 0;
    leftUs = rightUs = 1500;
    if (initialized) {
        leftDrum.writeMicroseconds(1500);
        rightDrum.writeMicroseconds(1500);
    }
    lastCommandMs = millis();
}

void drum_control_update() {
    if ((leftPercent || rightPercent) &&
        millis() - lastCommandMs > DRUM_COMMAND_TIMEOUT_MS) drum_control_stop();
}

int drum_control_left_percent() { return leftPercent; }
int drum_control_right_percent() { return rightPercent; }
int drum_control_left_us() { return leftUs; }
int drum_control_right_us() { return rightUs; }
bool drum_control_is_active() { return leftPercent || rightPercent; }
