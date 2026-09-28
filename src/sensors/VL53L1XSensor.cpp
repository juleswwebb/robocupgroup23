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
    delay(100);

    sensor_.setTimeout(500);
    if (!sensor_.init()) {
        Serial.print("VL53L1X '");
        Serial.print(name_);
        Serial.println("' failed to initialise (check wiring/power/bus)");
        // Keep a failed device isolated at 0x29 so it cannot collide with
        // the next sensor in the sequential address-assignment chain.
        expander_->digitalWrite(xshutPin_, LOW);
        initialized_ = false;
        return false;
    }

    // Move off the shared power-on default (0x29) so the next sensor in the
    // chain can be brought up without an address collision.
    sensor_.setAddress(address_);
    sensor_.setDistanceMode(mode_);
    // Match Group 7's proven L1 configuration for these directional sensors.
    sensor_.setROISize(8, 8);
    sensor_.setMeasurementTimingBudget(timingBudgetUs_);
    // Continuous, non-blocking acquisition: update() only consumes ready
    // samples while the sensor runs at the configured 50 ms cadence.
    sensor_.startContinuous(50);
    sensor_.setTimeout(5);
    initialized_ = true;
    return true;
}

void VL53L1XSensor::update() {
    if (!initialized_) {
        return; // never came up - don't block the scheduler on a dead sensor
    }
    // Always perform the readiness transaction before checking last_status.
    // Short-circuiting on a previous I2C error would prevent a later poll
    // from succeeding, leaving the sensor permanently marked invalid.
    const bool ready = sensor_.dataReady();
    if (sensor_.last_status != 0) {
        lastStatus_ = 255; // I2C failure: do not publish stale range data
        noReturn_ = false;
        return;
    }
    if (!ready) {
        return; // last measurement still in flight, cached value stands
    }

    sensor_.read(false);
    if (sensor_.last_status != 0) {
        lastStatus_ = 255; // I2C/read failure: don't publish stale range data
        noReturn_ = false;
        return;
    }

    lastRangeMM_ = sensor_.ranging_data.range_mm;
    const VL53L1X::RangeStatus status = sensor_.ranging_data.range_status;
    hasSample_ = true;
    lastSampleAt_ = millis();
    noReturn_ = status == VL53L1X::SignalFail;
    // Treat the clipped-min-range result as a valid close detection; the
    // library can report it with a zero distance, so clamp that to 1 mm.
    if (status == VL53L1X::RangeValidMinRangeClipped) {
        if (lastRangeMM_ == 0) lastRangeMM_ = 1;
        lastStatus_ = VL53L1X::RangeValid;
    } else {
        lastStatus_ = status;
    }
}
