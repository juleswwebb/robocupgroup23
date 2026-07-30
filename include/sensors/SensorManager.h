//************************************
//         SensorManager.h
//************************************
//
// Owns a set of DistanceSensor instances and gives one place to bring them
// all up and read any of them by name or index.

#ifndef SENSOR_MANAGER_H_
#define SENSOR_MANAGER_H_

#include "sensors/DistanceSensor.h"

#define SENSOR_MANAGER_MAX_SENSORS 16

class SensorManager {
public:
    // Register a sensor. For I2C sensors whose address gets reassigned at
    // boot (VL53L0X/VL53L1X), registration order is the bring-up order:
    // beginAll() brings sensors up one at a time in this order, so each
    // gets its address before the next is released from reset.
    bool addSensor(DistanceSensor* sensor);

    // Puts every sensor into standby (preReset), then brings each one up
    // and configures it in registration order. Returns true only if every
    // sensor's begin() succeeded.
    bool beginAll();

    // Poll every sensor once. Call this periodically (e.g. from a
    // scheduled task) rather than calling update() on each sensor yourself.
    void updateAll();

    DistanceSensor* get(const char* name) const;
    DistanceSensor* get(uint8_t index) const;
    uint8_t count() const { return count_; }

private:
    DistanceSensor* sensors_[SENSOR_MANAGER_MAX_SENSORS] = {nullptr};
    uint8_t count_ = 0;
};

#endif /* SENSOR_MANAGER_H_ */
