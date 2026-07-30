//************************************
//         VL53L1XSensor.h
//************************************

#ifndef VL53L1X_SENSOR_H_
#define VL53L1X_SENSOR_H_

#include <VL53L1X.h>
#include <Wire.h>
#include <SparkFunSX1509.h>
#include "sensors/DistanceSensor.h"

class VL53L1XSensor : public DistanceSensor {
public:
    // expander:  SX1509 IO expander this sensor's XSHUT line is wired through.
    // xshutPin:  SX1509 pin (IO0-IO15) wired to this sensor's XSHUT.
    // address:   I2C address to assign this sensor once it's brought out of reset.
    // bus:       which Teensy I2C bus this sensor itself is physically wired to.
    VL53L1XSensor(const char* name, SX1509* expander, uint8_t xshutPin, uint8_t address,
                  TwoWire* bus = &Wire,
                  VL53L1X::DistanceMode mode = VL53L1X::Short,
                  uint32_t timingBudgetUs = 50000);

    bool begin() override;
    void preReset() override;
    void update() override;
    uint16_t getDistanceMM() const override { return lastRangeMM_; }
    // False if begin() never succeeded, or the last reading wasn't a valid range.
    bool isValid() const override { return initialized_ && lastStatus_ == 0; }
    const char* getName() const override { return name_; }

private:
    const char* name_;
    SX1509* expander_;
    uint8_t xshutPin_;
    uint8_t address_;
    VL53L1X::DistanceMode mode_;
    uint32_t timingBudgetUs_;
    VL53L1X sensor_;
    uint16_t lastRangeMM_ = 0;
    uint8_t lastStatus_ = 255;
    bool initialized_ = false;
};

#endif /* VL53L1X_SENSOR_H_ */
