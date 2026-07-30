//************************************
//         DistanceSensors.h
//************************************
//
// Task-scheduler-facing entry points for the TOF sensor subsystem. Owns
// the actual SensorManager and sensor instances internally (see
// DistanceSensors.cpp) - callers just get plain functions to init/poll,
// plus lookup by name for reading any one sensor.

#ifndef DISTANCE_SENSORS_H_
#define DISTANCE_SENSORS_H_

#include "sensors/DistanceSensor.h"

// Bring up I2C, then every registered sensor (XSHUT sequencing + address
// assignment for the VL53L0X/L1X sensors). Call once from setup().
void distance_sensors_init();

// Poll every sensor once. Call periodically from a scheduled task.
void distance_sensors_update();

// Print every sensor's name + latest reading to Serial. For debugging.
void distance_sensors_print();

// Fetch any sensor by the name it was registered with, e.g. "tof_L0_0".
// Returns nullptr if no sensor has that name.
DistanceSensor* distance_sensor_get(const char* name);

#endif /* DISTANCE_SENSORS_H_ */
