#include "sensors/ImuSensor.h"
#include <Arduino.h>

ImuSensor::ImuSensor(const char* name, uint8_t address, TwoWire* bus, int32_t sensorId)
    : name_(name), sensor_(sensorId, address, bus) {}

bool ImuSensor::begin() {
    initialized_ = sensor_.begin();
    if (!initialized_) {
        Serial.print("IMU '");
        Serial.print(name_);
        Serial.println("' failed to initialise (check wiring/address/bus)");
        return false;
    }

    delay(1000); // let it settle before the first read, matches the vendor example
    sensor_.setExtCrystalUse(true);
    return true;
}

void ImuSensor::update() {
    if (!initialized_) {
        return;
    }

    sensors_event_t orientationData;
    sensor_.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    heading_ = orientationData.orientation.x;
    pitch_ = orientationData.orientation.y;
    roll_ = orientationData.orientation.z;

    sensor_.getCalibration(&calSystem_, &calGyro_, &calAccel_, &calMag_);
}
