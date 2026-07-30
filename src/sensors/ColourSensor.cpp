#include "sensors/ColourSensor.h"
#include <Arduino.h>

ColourSensor::ColourSensor(const char* name, uint8_t address, TwoWire* bus,
                            uint8_t integrationTime, tcs34725Gain_t gain)
    : name_(name), address_(address), bus_(bus), sensor_(integrationTime, gain) {}

bool ColourSensor::begin() {
    initialized_ = sensor_.begin(address_, bus_);
    if (!initialized_) {
        Serial.print("Colour sensor '");
        Serial.print(name_);
        Serial.println("' failed to initialise (check wiring/address/bus)");
    }
    return initialized_;
}

void ColourSensor::update() {
    if (!initialized_) {
        return;
    }
    sensor_.getRawData(&red_, &green_, &blue_, &clear_);
}
