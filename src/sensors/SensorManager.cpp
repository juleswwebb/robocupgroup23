#include "sensors/SensorManager.h"
#include <Arduino.h>
#include <string.h>

bool SensorManager::addSensor(DistanceSensor* sensor) {
    if (count_ >= SENSOR_MANAGER_MAX_SENSORS) {
        Serial.println("SensorManager: too many sensors registered, dropping one");
        return false;
    }
    sensors_[count_++] = sensor;
    return true;
}

bool SensorManager::beginAll() {
    for (uint8_t i = 0; i < count_; i++) {
        sensors_[i]->preReset();
    }

    bool allOk = true;
    for (uint8_t i = 0; i < count_; i++) {
        if (!sensors_[i]->begin()) {
            allOk = false;
        }
    }
    return allOk;
}

void SensorManager::updateAll() {
    for (uint8_t i = 0; i < count_; i++) {
        sensors_[i]->update();
    }
}

DistanceSensor* SensorManager::get(const char* name) const {
    for (uint8_t i = 0; i < count_; i++) {
        if (strcmp(sensors_[i]->getName(), name) == 0) {
            return sensors_[i];
        }
    }
    return nullptr;
}

DistanceSensor* SensorManager::get(uint8_t index) const {
    if (index >= count_) {
        return nullptr;
    }
    return sensors_[index];
}
