//************************************
//         ImuSensor.h
//************************************
//
// Wraps the BNO055 orientation sensor (the IMU half of the SEN0253
// IMU+barometer combo board - the BMP280 barometer isn't handled here).
// Runs in the default NDOF fusion mode, which gives absolute heading/
// roll/pitch directly without doing manual sensor fusion ourselves.
//
// Calibration (system/gyro/accel/mag, each 0-3) starts at 0 on power-up
// and needs the sensor moved through a few orientations before readings
// are reliable - see getSystemCalibration() etc. A system calibration of
// 0 means "don't trust this reading yet".

#ifndef IMU_SENSOR_H_
#define IMU_SENSOR_H_

#include <stdint.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

class ImuSensor {
public:
    ImuSensor(const char* name, uint8_t address = 0x28, TwoWire* bus = &Wire,
              int32_t sensorId = -1);

    bool begin();
    void update(); // reads orientation + calibration status

    float getHeading() const { return heading_; } // degrees, 0-360
    float getRoll() const { return roll_; }        // degrees
    float getPitch() const { return pitch_; }       // degrees

    uint8_t getSystemCalibration() const { return calSystem_; }
    uint8_t getGyroCalibration() const { return calGyro_; }
    uint8_t getAccelCalibration() const { return calAccel_; }
    uint8_t getMagCalibration() const { return calMag_; }

    bool isValid() const { return initialized_; }
    const char* getName() const { return name_; }

private:
    const char* name_;
    Adafruit_BNO055 sensor_;
    float heading_ = 0;
    float roll_ = 0;
    float pitch_ = 0;
    uint8_t calSystem_ = 0;
    uint8_t calGyro_ = 0;
    uint8_t calAccel_ = 0;
    uint8_t calMag_ = 0;
    bool initialized_ = false;
};

#endif /* IMU_SENSOR_H_ */
