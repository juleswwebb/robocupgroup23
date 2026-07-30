//************************************
//         IRDistanceSensor.h
//************************************
//
// Analog Sharp-style IR distance sensor (GP2Y0Axxx family). These output a
// voltage that's roughly inversely related to distance; the exact curve
// differs by model (short/medium/long range), but the wiring (Vo/GND/Vcc)
// and general shape are the same across the family.
//
// getDistanceMM() applies a power-law approximation of that curve
// (distance_cm = scaleNumerator * voltage^exponent). The defaults are a
// commonly-used fit for the medium-range GP2Y0A21YK - if you know each
// sensor's exact model, or want real accuracy for the design report,
// recalibrate scaleNumerator/exponent per-sensor by measuring voltage at a
// few known distances and re-fitting. getVoltage()/getRawADC() expose the
// raw reading for that purpose.

#ifndef IR_DISTANCE_SENSOR_H_
#define IR_DISTANCE_SENSOR_H_

#include <stdint.h>
#include "sensors/DistanceSensor.h"

class IRDistanceSensor : public DistanceSensor {
public:
    IRDistanceSensor(const char* name, uint8_t analogPin,
                      float scaleNumerator = 27.86f, float exponent = -1.15f,
                      float adcRefVoltage = 3.3f, uint16_t adcMaxCounts = 1023);

    bool begin() override;
    void update() override;
    uint16_t getDistanceMM() const override { return lastDistanceMM_; }
    // False when the voltage is too low for the curve fit to be meaningful
    // (out of range / no reflective object in front of the sensor).
    bool isValid() const override { return lastValid_; }
    const char* getName() const override { return name_; }

    float getVoltage() const { return lastVoltage_; }
    uint16_t getRawADC() const { return lastRawADC_; }

private:
    const char* name_;
    uint8_t analogPin_;
    float scaleNumerator_;
    float exponent_;
    float adcRefVoltage_;
    uint16_t adcMaxCounts_;
    uint16_t lastRawADC_ = 0;
    float lastVoltage_ = 0.0f;
    uint16_t lastDistanceMM_ = 0;
    bool lastValid_ = false;
};

#endif /* IR_DISTANCE_SENSOR_H_ */
