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
    // Release this sensor from reset. Wait for it to boot before talking to it.
    expander_->digitalWrite(xshutPin_, HIGH);
    delay(10);

    sensor_.setTimeout(500);
    if (!sensor_.init()) {
        Serial.print("VL53L0X '");
        Serial.print(name_);
        Serial.println("' failed to initialise (check wiring/power/bus)");
        initialized_ = false;
        return false;
    }

    // Move off the shared power-on default (0x29) so the next sensor in the
    // chain can be brought up without an address collision.
    sensor_.setAddress(address_);
    sensor_.setMeasurementTimingBudget(timingBudgetUs_);
    sensor_.startContinuous();
    initialized_ = true;
    return true;
}

void VL53L0XSensor::update() {
    if (!initialized_) {
        return; // never came up - don't block the scheduler on a dead sensor
    }
    lastRangeMM_ = sensor_.readRangeContinuousMillimeters();
    lastTimeout_ = sensor_.timeoutOccurred();
}
