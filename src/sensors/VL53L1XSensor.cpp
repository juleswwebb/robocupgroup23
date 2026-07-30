#include "sensors/VL53L1XSensor.h"
#include <Arduino.h>

VL53L1XSensor::VL53L1XSensor(const char* name, SX1509* expander, uint8_t xshutPin,
                              uint8_t address, TwoWire* bus, VL53L1X::DistanceMode mode,
                              uint32_t timingBudgetUs)
    : name_(name), expander_(expander), xshutPin_(xshutPin), address_(address), mode_(mode),
      timingBudgetUs_(timingBudgetUs) {
    sensor_.setBus(bus);
}

void VL53L1XSensor::preReset() {
    expander_->pinMode(xshutPin_, OUTPUT);
    expander_->digitalWrite(xshutPin_, LOW);
}

bool VL53L1XSensor::begin() {
    // Release this sensor from reset. Wait for it to boot before talking to it.
    expander_->digitalWrite(xshutPin_, HIGH);
    delay(10);

    sensor_.setTimeout(500);
    if (!sensor_.init()) {
        Serial.print("VL53L1X '");
        Serial.print(name_);
        Serial.println("' failed to initialise (check wiring/power/bus)");
        initialized_ = false;
        return false;
    }

    // Move off the shared power-on default (0x29) so the next sensor in the
    // chain can be brought up without an address collision.
    sensor_.setAddress(address_);
    sensor_.setDistanceMode(mode_);
    sensor_.setMeasurementTimingBudget(timingBudgetUs_);

    // Kick off the first measurement. `false` = don't block waiting for it;
    // update() picks the result up via dataReady()/read() below.
    sensor_.readRangeSingleMillimeters(false);
    initialized_ = true;
    return true;
}

void VL53L1XSensor::update() {
    if (!initialized_) {
        return; // never came up - don't block the scheduler on a dead sensor
    }
    if (!sensor_.dataReady()) {
        return; // last measurement still in flight, cached value stands
    }

    sensor_.read(false);
    lastRangeMM_ = sensor_.ranging_data.range_mm;
    lastStatus_ = sensor_.ranging_data.range_status;

    sensor_.readRangeSingleMillimeters(false); // trigger the next measurement
}
