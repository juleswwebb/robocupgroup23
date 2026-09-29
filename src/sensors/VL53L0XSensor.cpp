#include "sensors/VL53L0XSensor.h"
#include <Arduino.h>

VL53L0XSensor::VL53L0XSensor(const char* name, SX1509* expander, uint8_t xshutPin,
                              uint8_t address, TwoWire* bus, uint32_t timingBudgetUs)
    : name_(name), expander_(expander), xshutPin_(xshutPin), address_(address),
      timingBudgetUs_(timingBudgetUs) {
    sensor_.setBus(bus);
}

void VL53L0XSensor::preReset() {
    expander_->pinMode(xshutPin_, OUTPUT);
    expander_->digitalWrite(xshutPin_, LOW);
}

bool VL53L0XSensor::begin() {
    static const uint8_t maxAttempts = 2;
    initialized_ = false;
    hasSample_ = false;
    lastTimeout_ = true;
    noReturn_ = false;
    lastStatus_ = 255;
    modelId_ = 0;

    for (initAttempts_ = 1; initAttempts_ <= maxAttempts; initAttempts_++) {
        // Power-cycle this sensor between attempts while all previously
        // initialized devices remain at their assigned unique addresses.
        expander_->pinMode(xshutPin_, OUTPUT);
        expander_->digitalWrite(xshutPin_, LOW);
        delay(20);
        expander_->digitalWrite(xshutPin_, HIGH);
        delay(100);

        sensor_.setTimeout(500);
        if (!sensor_.init()) {
            // Capture what answered at the default address: a mismatched
            // module type has I2C status 0 but a non-matching model ID.
            modelId_ = sensor_.readReg(VL53L0X::IDENTIFICATION_MODEL_ID);
            const uint8_t i2cStatus = sensor_.last_status;
            Serial.print("VL53L0X '"); Serial.print(name_);
            Serial.print("' init attempt "); Serial.print(initAttempts_);
            Serial.print("/"); Serial.print(maxAttempts);
            Serial.print(" failed; model=0x"); Serial.print(modelId_, HEX);
            Serial.print(" i2c="); Serial.println(i2cStatus);
            expander_->digitalWrite(xshutPin_, LOW);
            delay(20);
            continue;
        }

        modelId_ = 0x00EE;

        // Move off the shared power-on default (0x29) so the next sensor in
        // the chain can be brought up without an address collision.
        sensor_.setAddress(address_);
        sensor_.setMeasurementTimingBudget(timingBudgetUs_);
        sensor_.startContinuous();
        sensor_.setTimeout(5);
        initialized_ = true;
        return true;
    }

    initAttempts_ = maxAttempts;
    expander_->digitalWrite(xshutPin_, LOW);
    return false;
}

void VL53L0XSensor::update() {
    if (!initialized_) {
        return; // never came up - don't block the scheduler on a dead sensor
    }
    const uint8_t ready = sensor_.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (sensor_.last_status != 0 || !(ready & 7)) {
        return;
    }

    const uint8_t rangeStatus = (sensor_.readReg(VL53L0X::RESULT_RANGE_STATUS) & 0x78) >> 3;
    if (sensor_.last_status != 0) {
        lastStatus_ = 255;
        hasSample_ = true;
        lastSampleAt_ = millis();
        return;
    }

    lastRangeMM_ = sensor_.readRangeContinuousMillimeters();
    lastStatus_ = rangeStatus;
    lastTimeout_ = sensor_.timeoutOccurred() || sensor_.last_status != 0;
    noReturn_ = !lastTimeout_ && rangeStatus == 4;
    hasSample_ = true;
    lastSampleAt_ = millis();

    if (!lastTimeout_ && rangeStatus == 11 && lastRangeMM_ > 0 && lastRangeMM_ < 8190) {
        lastStatus_ = 0;
    } else if (!noReturn_ && !lastTimeout_) {
        lastTimeout_ = true;
    }
}
