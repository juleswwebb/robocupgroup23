#include "Navigation.h"

#include "DistanceSensors.h"
#include "DriveControl.h"
#include "Encoders.h"
#include "IMU.h"
#include <Arduino.h>
#include <math.h>

namespace {
enum class State : uint8_t { Idle, Forward, Turning };

State state = State::Idle;
bool active = false;
int driveSpeed = 30;
int turnSpeed = 25;
int frontStopMm = 300;
uint16_t frontMm = 0;
uint16_t leftMm = 0;
uint16_t rightMm = 0;
float targetHeading = 0.0f;
unsigned long stateStartedMs = 0;
uint8_t blockedFrames = 0;
char stopReason[64] = "Not started";

float heading_error(float current, float target) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

uint16_t minimum_valid(const uint16_t* grid, uint8_t rowStart,
                       uint8_t rowEnd, uint8_t colStart, uint8_t colEnd) {
    uint16_t result = 0;
    for (uint8_t row = rowStart; row < rowEnd; ++row) {
        for (uint8_t col = colStart; col < colEnd; ++col) {
            const uint16_t value = grid[row * 8 + col];
            if (value == 0 || value >= 4000) continue;
            if (result == 0 || value < result) result = value;
        }
    }
    return result;
}

bool update_ranges() {
    uint16_t grid[64];
    if (!distance_sensors_get_8x8_grid(grid)) {
        frontMm = leftMm = rightMm = 0;
        return false;
    }

    // Ignore the bottom two rows, which commonly see the floor/chassis. The
    // central four columns form the forward collision gate; the side halves
    // choose the clearer direction for a 90-degree avoidance turn.
    frontMm = minimum_valid(grid, 0, 6, 2, 6);
    leftMm = minimum_valid(grid, 0, 6, 0, 4);
    rightMm = minimum_valid(grid, 0, 6, 4, 8);
    return frontMm != 0;
}

void copy_reason(const char* reason) {
    if (reason == nullptr || *reason == '\0') reason = "Stopped";
    strncpy(stopReason, reason, sizeof(stopReason) - 1);
    stopReason[sizeof(stopReason) - 1] = '\0';
}
}

void navigation_init() {
    active = false;
    state = State::Idle;
    copy_reason("Not started");
}

bool navigation_set_enabled(bool enabled) {
    if (!enabled) {
        navigation_stop("Stopped by operator");
        return true;
    }
    if (!imu_is_valid()) {
        navigation_stop("Cannot start: IMU invalid");
        return false;
    }
    if (!update_ranges()) {
        navigation_stop("Cannot start: no valid forward 8x8 range");
        return false;
    }
    active = true;
    state = State::Forward;
    blockedFrames = 0;
    stateStartedMs = millis();
    copy_reason("Running");
    return true;
}

void navigation_stop(const char* reason) {
    active = false;
    state = State::Idle;
    blockedFrames = 0;
    drive_control_stop();
    copy_reason(reason);
}

void navigation_update() {
    if (!active) return;
    const unsigned long now = millis();
    if (!imu_is_valid()) {
        navigation_stop("IMU lost");
        return;
    }
    if (!update_ranges()) {
        navigation_stop("Forward ranging lost");
        return;
    }

    if (state == State::Forward) {
        if (frontMm <= static_cast<uint16_t>(frontStopMm)) {
            if (blockedFrames < 3) ++blockedFrames;
        } else {
            blockedFrames = 0;
        }
        if (blockedFrames >= 2) {
            const bool turnRight = rightMm == 0 || (leftMm != 0 && rightMm > leftMm);
            targetHeading = imu_get_heading() + (turnRight ? 90.0f : -90.0f);
            while (targetHeading >= 360.0f) targetHeading -= 360.0f;
            while (targetHeading < 0.0f) targetHeading += 360.0f;
            state = State::Turning;
            stateStartedMs = now;
            blockedFrames = 0;
        } else {
            drive_control_set_percent(driveSpeed, driveSpeed);
        }
        return;
    }

    if (state == State::Turning) {
        const float error = heading_error(imu_get_heading(), targetHeading);
        if (fabsf(error) <= 5.0f) {
            drive_control_stop();
            state = State::Forward;
            stateStartedMs = now;
            return;
        }
        if (now - stateStartedMs > 5000UL) {
            navigation_stop("Turn timeout");
            return;
        }
        const int direction = error > 0.0f ? 1 : -1;
        drive_control_set_percent(direction * turnSpeed, -direction * turnSpeed);
    }
}

bool navigation_is_active() { return active; }
const char* navigation_get_state_name() {
    switch (state) {
        case State::Forward: return "FORWARD";
        case State::Turning: return "TURNING";
        default: return "IDLE";
    }
}
const char* navigation_get_stop_reason() { return stopReason; }
uint16_t navigation_get_front_mm() { return frontMm; }
uint16_t navigation_get_left_mm() { return leftMm; }
uint16_t navigation_get_right_mm() { return rightMm; }
float navigation_get_target_heading() { return targetHeading; }

void navigation_set_speed_percent(int value) { driveSpeed = constrain(value, 5, 60); }
void navigation_set_turn_percent(int value) { turnSpeed = constrain(value, 5, 60); }
void navigation_set_front_stop_mm(int value) { frontStopMm = constrain(value, 100, 1500); }
int navigation_get_speed_percent() { return driveSpeed; }
int navigation_get_turn_percent() { return turnSpeed; }
int navigation_get_front_stop_mm() { return frontStopMm; }
