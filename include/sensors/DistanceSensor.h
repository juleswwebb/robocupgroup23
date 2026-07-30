//************************************
//         DistanceSensor.h
//************************************
//
// Common interface every range-finding sensor implements, so the rest of
// the codebase can read "distance in mm" without caring whether it's
// talking to a VL53L0X, VL53L1X, the 8x8 TOF array, or a serial TOF module.

#ifndef DISTANCE_SENSOR_H_
#define DISTANCE_SENSOR_H_

#include <stdint.h>

class DistanceSensor {
public:
    virtual ~DistanceSensor() {}

    // One-time hardware bring-up (I2C address assignment, sensor config).
    // Returns true on success.
    virtual bool begin() = 0;

    // Called once per sensor, before any begin(), for sensors that need to
    // be held in reset so they don't collide with others on the same I2C
    // bus (see SensorManager::beginAll). No-op for sensors that don't need it.
    virtual void preReset() {}

    // Poll the sensor and refresh the cached reading. Call this
    // periodically (e.g. from a scheduled task) - NOT from getDistanceMM().
    virtual void update() = 0;

    // Last cached reading in millimetres. Cheap, non-blocking.
    virtual uint16_t getDistanceMM() const = 0;

    // False if the last update() didn't produce a valid reading
    // (sensor timeout, no target in range, bad checksum, etc.)
    virtual bool isValid() const = 0;

    virtual const char* getName() const = 0;
};

#endif /* DISTANCE_SENSOR_H_ */
