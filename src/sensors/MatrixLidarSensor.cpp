#include "sensors/MatrixLidarSensor.h"
#include <Arduino.h>
#include <string.h>

MatrixLidarSensor::MatrixLidarSensor(const char* name, uint8_t address, TwoWire* bus)
    : name_(name), sensor_(address, bus) {}

bool MatrixLidarSensor::begin() {
    // By the time this runs, several other sensors on other buses have
    // already gone through their own bring-up sequence; a transient bus
    // hiccup can make the very first attempt fail even though the sensor
    // is fine, so retry a few times before giving up for good.
    const uint8_t maxAttempts = 5;
    for (uint8_t attempt = 1; attempt <= maxAttempts; attempt++) {
        if (sensor_.begin() == 0 && sensor_.setRangingMode(eMatrix_8X8) == 0) {
            initialized_ = true;
            return true;
        }
        delay(20);
    }

    Serial.print("8x8 TOF array '");
    Serial.print(name_);
    Serial.println("' failed to initialise after retries (check wiring/address/bus)");
    initialized_ = false;
    return false;
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
