#include "DriveControl.h"
#include "sensor_config.h"

#include <Arduino.h>
#include <Servo.h>

static Servo leftDrive;
static Servo rightDrive;

static int leftPercent = 0;
static int rightPercent = 0;
static int maxPercent = DRIVE_DEFAULT_MAX_PERCENT;
static int leftMicroseconds = DRIVE_NEUTRAL_US;
static int rightMicroseconds = DRIVE_NEUTRAL_US;
static bool active = false;
static unsigned long lastCommandMs = 0;

static int clamp_percent(int value) {
    value = constrain(value, -100, 100);
    return constrain(value, -maxPercent, maxPercent);
}

static int percent_to_microseconds(int percent, bool reversed) {
    // The ESC calibration supplied with the robot uses 1050/1500/1950 us.
    if (reversed) percent = -percent;
    return constrain(DRIVE_NEUTRAL_US + percent * DRIVE_US_PER_PERCENT,
                     DRIVE_MIN_US, DRIVE_MAX_US);
}

void drive_control_init() {
    leftDrive.attach(DRIVE_LEFT_PIN);
    rightDrive.attach(DRIVE_RIGHT_PIN);
    drive_control_stop();
}

void drive_control_set_percent(int left_percent, int right_percent) {
    leftPercent = clamp_percent(left_percent);
    rightPercent = clamp_percent(right_percent);
    leftMicroseconds = percent_to_microseconds(leftPercent, DRIVE_LEFT_REVERSED);
    rightMicroseconds = percent_to_microseconds(rightPercent, DRIVE_RIGHT_REVERSED);
    leftDrive.writeMicroseconds(leftMicroseconds);
    rightDrive.writeMicroseconds(rightMicroseconds);
    active = (leftPercent != 0 || rightPercent != 0);
    lastCommandMs = millis();
}

void drive_control_stop() {
    leftPercent = 0;
    rightPercent = 0;
    leftMicroseconds = DRIVE_NEUTRAL_US;
    rightMicroseconds = DRIVE_NEUTRAL_US;
    leftDrive.writeMicroseconds(leftMicroseconds);
    rightDrive.writeMicroseconds(rightMicroseconds);
    active = false;
    lastCommandMs = millis();
}

void drive_control_update() {
    // A lost USB link, crashed app, or missed key-release must stop the robot.
    if (active && millis() - lastCommandMs > DRIVE_COMMAND_TIMEOUT_MS) {
        drive_control_stop();
    }
}

void drive_control_set_max_percent(int requested) {
    maxPercent = constrain(requested, 0, DRIVE_HARD_MAX_PERCENT);
    // Immediately apply a reduced limit to an existing command too.
    drive_control_set_percent(leftPercent, rightPercent);
}

int drive_control_get_max_percent() { return maxPercent; }
int drive_control_get_left_percent() { return leftPercent; }
int drive_control_get_right_percent() { return rightPercent; }
int drive_control_get_left_microseconds() { return leftMicroseconds; }
int drive_control_get_right_microseconds() { return rightMicroseconds; }
bool drive_control_is_active() { return active; }
