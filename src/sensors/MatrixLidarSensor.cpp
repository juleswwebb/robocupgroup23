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
            // The vendor default is 8 seconds. A missed frame must never stall
            // the console, drive watchdog, or Bluetooth link for that long.
            sensor_.setTimeout(150);
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

    // The array does not produce a fresh 8x8 frame every 20 ms. Polling it at
    // the manager rate overwhelms the module and makes timeouts much likelier.
    const uint32_t now = millis();
    if (now - lastPollMs_ < 100) {
        return;
    }
    lastPollMs_ = now;
    lastReadOk_ = (sensor_.getAllData(grid_) == 0);
    if (lastReadOk_) lastSuccessfulReadMs_ = now;
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
