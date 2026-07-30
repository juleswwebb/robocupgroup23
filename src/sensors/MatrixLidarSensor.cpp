#include "sensors/MatrixLidarSensor.h"
#include <Arduino.h>
#include <string.h>

MatrixLidarSensor::MatrixLidarSensor(const char* name, uint8_t address, TwoWire* bus)
    : name_(name), sensor_(address, bus) {}

bool MatrixLidarSensor::begin() {
    if (sensor_.begin() != 0) {
        Serial.print("8x8 TOF array '");
        Serial.print(name_);
        Serial.println("' failed to initialise (check wiring/address/bus)");
        initialized_ = false;
        return false;
    }
    if (sensor_.setRangingMode(eMatrix_8X8) != 0) {
        Serial.print("8x8 TOF array '");
        Serial.print(name_);
        Serial.println("' failed to set ranging mode");
        initialized_ = false;
        return false;
    }
    initialized_ = true;
    return true;
}

void MatrixLidarSensor::update() {
    if (!initialized_) {
        return; // never came up - don't block the scheduler on a dead sensor
    }
    lastReadOk_ = (sensor_.getAllData(grid_) == 0);
}

uint16_t MatrixLidarSensor::getDistanceMM() const {
    return grid_[3 * 8 + 3]; // near-centre pixel
}

bool MatrixLidarSensor::isValid() const {
    return initialized_ && lastReadOk_ && getDistanceMM() < MATRIX_LIDAR_NO_TARGET_MM;
}

void MatrixLidarSensor::getGrid(uint16_t* buf) const {
    memcpy(buf, grid_, sizeof(grid_));
}
