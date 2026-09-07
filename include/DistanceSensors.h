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

// Print the 8x8 array's full 64-pixel grid (row by row, mm) to Serial.
// The generic distance_sensors_print() above only shows one pixel per
// sensor, which isn't enough to see what this sensor's actually seeing.
void distance_sensors_print_8x8_grid();

// Fetch any sensor by the name it was registered with, e.g. "tof_L0_0".
// Returns nullptr if no sensor has that name.
DistanceSensor* distance_sensor_get(const char* name);

// Iterate every registered sensor (for telemetry, logging, etc.) without
// having to know their names up front.
unsigned char distance_sensors_count();
DistanceSensor* distance_sensor_get_by_index(unsigned char index);

// Closest pixel anywhere in the 8x8 array's field of view, in mm - more
// useful for obstacle avoidance than the single centre pixel that the
// generic DistanceSensor interface exposes. Returns 0 if nothing is in range.
unsigned short distance_sensors_8x8_min_mm();

#endif /* DISTANCE_SENSORS_H_ */
