#include "sensors/InductiveSensor.h"
#include <Arduino.h>

InductiveSensor::InductiveSensor(const char* name, uint8_t pin, unsigned long debounceDelayMs)
    : name_(name), pin_(pin), debounceDelayMs_(debounceDelayMs) {}

bool InductiveSensor::begin() {
    pinMode(pin_, INPUT);
    return true;
}

void InductiveSensor::update() {
    bool reading = digitalRead(pin_);
    rawState_ = reading;

    if (reading != lastReading_) {
        lastDebounceTime_ = millis();
    }

    if ((millis() - lastDebounceTime_) > debounceDelayMs_) {
        if (reading != currentState_) {
            currentState_ = reading;
            detected_ = !currentState_; // inverted - see class comment

            if (detected_) {
                detectionCount_++;
                lastDetectionTime_ = millis();
            }
        }
    }

    lastReading_ = reading;
}
