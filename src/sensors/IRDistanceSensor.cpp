#include "sensors/IRDistanceSensor.h"
#include <Arduino.h>
#include <math.h>

IRDistanceSensor::IRDistanceSensor(const char* name, uint8_t analogPin,
                                    float scaleNumerator, float exponent,
                                    float adcRefVoltage, uint16_t adcMaxCounts)
    : name_(name), analogPin_(analogPin), scaleNumerator_(scaleNumerator),
      exponent_(exponent), adcRefVoltage_(adcRefVoltage), adcMaxCounts_(adcMaxCounts) {}

bool IRDistanceSensor::begin() {
    pinMode(analogPin_, INPUT);
    return true;
}

void IRDistanceSensor::update() {
    lastRawADC_ = analogRead(analogPin_);
    lastVoltage_ = (lastRawADC_ / (float)adcMaxCounts_) * adcRefVoltage_;

    // Below ~0.1V the curve is flat/noisy (out of range, or nothing
    // reflective in front of the sensor) and the power-law fit blows up
    // as voltage -> 0, so treat it as no reading rather than a nonsense one.
    if (lastVoltage_ < 0.1f) {
        lastValid_ = false;
        return;
    }

    float distanceCm = scaleNumerator_ * powf(lastVoltage_, exponent_);
    lastDistanceMM_ = (uint16_t)(distanceCm * 10.0f);
    lastValid_ = true;
}
