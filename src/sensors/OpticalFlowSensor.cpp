#include "sensors/OpticalFlowSensor.h"
#include <Arduino.h>

OpticalFlowSensor::OpticalFlowSensor(const char* name, uint8_t csPin)
    : name_(name), sensor_(csPin) {}

bool OpticalFlowSensor::begin() {
    initialized_ = sensor_.begin();
    if (!initialized_) {
        Serial.print("Optical flow sensor '");
        Serial.print(name_);
        Serial.println("' failed to initialise (check wiring/CS pin)");
    }
    return initialized_;
}

void OpticalFlowSensor::update() {
    if (!initialized_) {
        return;
    }
    sensor_.readMotionCount(&deltaX_, &deltaY_);
    totalX_ += deltaX_;
    totalY_ += deltaY_;
}
