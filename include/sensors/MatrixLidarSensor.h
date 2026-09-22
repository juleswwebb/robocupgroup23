//************************************
//         MatrixLidarSensor.h
//************************************
//
// Wraps the DFRobot 8x8 array TOF (DFRobot_MatrixLidar, I2C mode).
// getDistanceMM() exposes the centre pixel so it fits the common
// DistanceSensor interface; getGrid() exposes the full 8x8 field for
// callers that want the whole picture (e.g. finding the nearest obstacle
// across the sensor's field of view).

#ifndef MATRIX_LIDAR_SENSOR_H_
#define MATRIX_LIDAR_SENSOR_H_

#include <DFRobot_MatrixLidar.h>
#include <Wire.h>
#include "sensors/DistanceSensor.h"

#define MATRIX_LIDAR_NO_TARGET_MM 4000 // this sensor's "nothing in range" code
#define MATRIX_LIDAR_GRID_SIZE 64

class MatrixLidarSensor : public DistanceSensor {
public:
    // bus: which Teensy I2C bus this sensor is physically wired to.
    explicit MatrixLidarSensor(const char* name, uint8_t address = 0x33, TwoWire* bus = &Wire);

    bool begin() override;
    void update() override;
    uint16_t getDistanceMM() const override; // centre pixel, mm
    bool isValid() const override;
    bool isGridAvailable() const { return initialized_ && lastReadOk_; }
    const char* getName() const override { return name_; }

    // Full 8x8 grid, row-major, mm. buf must hold MATRIX_LIDAR_GRID_SIZE entries.
    void getGrid(uint16_t* buf) const;

private:
    const char* name_;
    DFRobot_MatrixLidar_I2C sensor_;
    uint16_t grid_[MATRIX_LIDAR_GRID_SIZE] = {0};
    uint32_t lastPollMs_ = 0;
    bool lastReadOk_ = false;
    bool initialized_ = false;
};

#endif /* MATRIX_LIDAR_SENSOR_H_ */
