//************************************
//         SerialTofSensor.h
//************************************
//
// Wraps a TFmini-style serial TOF module (9-byte frames, header 0x59 0x59).
// Distance is reported in mm to stay consistent with the other
// DistanceSensor implementations (the module's native units are cm).

#ifndef SERIAL_TOF_SENSOR_H_
#define SERIAL_TOF_SENSOR_H_

#include <Arduino.h>
#include "sensors/DistanceSensor.h"

class SerialTofSensor : public DistanceSensor {
public:
    SerialTofSensor(const char* name, HardwareSerial& port, uint32_t baud = 115200);

    bool begin() override;
    void update() override;
    uint16_t getDistanceMM() const override { return lastRangeMM_; }
    bool isValid() const override { return frameReceived_; }
    const char* getName() const override { return name_; }

    uint16_t getSignalStrength() const { return lastStrength_; }

private:
    const char* name_;
    HardwareSerial& port_;
    uint32_t baud_;
    uint8_t rx_[9] = {0};
    uint8_t rxIndex_ = 0;
    uint16_t lastRangeMM_ = 0;
    uint16_t lastStrength_ = 0;
    bool frameReceived_ = false;
};

#endif /* SERIAL_TOF_SENSOR_H_ */
