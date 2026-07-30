//************************************
//         IMU.h
//************************************
//
// Task-scheduler-facing entry points for the IMU (BNO055). Owns the
// actual ImuSensor instance internally (see IMU.cpp).

#ifndef IMU_H_
#define IMU_H_

// Bring up the sensor. Call once from setup(). Blocks for ~1s while the
// sensor settles (matches the vendor example) - one-time cost at startup.
void imu_init();

// Poll the sensor once. Call periodically from a scheduled task.
void imu_update();

// Print heading/roll/pitch + calibration status to Serial. For debugging.
void imu_print();

#endif /* IMU_H_ */
