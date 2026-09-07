//************************************
//         IMU.h
//************************************
//
// Task-scheduler-facing entry points for the IMU (BNO055). Owns the
// actual ImuSensor instance internally (see IMU.cpp).

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

// Bring up the sensor. Call once from setup(). Blocks for ~1s while the
// sensor settles (matches the vendor example) - one-time cost at startup.
void imu_init();

// Poll the sensor once. Call periodically from a scheduled task.
void imu_update();

// Print heading/roll/pitch + calibration status to Serial. For debugging.
void imu_print();

// Latest orientation, degrees. Only meaningful when imu_is_valid().
float imu_get_heading();
float imu_get_roll();
float imu_get_pitch();

// Calibration status, 0-3 each. 0 for system means don't trust the readings.
uint8_t imu_get_system_calibration();
uint8_t imu_get_gyro_calibration();
uint8_t imu_get_accel_calibration();
uint8_t imu_get_mag_calibration();

bool imu_is_valid();

#endif /* IMU_H_ */
