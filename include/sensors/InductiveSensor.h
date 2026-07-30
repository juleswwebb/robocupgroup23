//************************************
//         InductiveSensor.h
//************************************
//
// LJ18A3-8-Z/BY PNP inductive proximity sensor (metal detection, ~8mm
// range). It's a 6-36V PNP output, well above the Teensy's 3.3V logic, so
// the kit's interface board between the sensor and this GPIO pin must
// already be doing the level-shifting - the wiring also inverts the
// signal in the process (confirmed working pattern: pin reads HIGH at
// rest, LOW when metal is detected). Debounced the same way as the known-
// working test sketch this was ported from.

#ifndef INDUCTIVE_SENSOR_H_
#define INDUCTIVE_SENSOR_H_

#include <stdint.h>

class InductiveSensor {
public:
    InductiveSensor(const char* name, uint8_t pin, unsigned long debounceDelayMs = 10);

    bool begin();
    void update();

    bool isDetected() const { return detected_; }
    bool getRawPinState() const { return rawState_; }
    unsigned long getDetectionCount() const { return detectionCount_; }
    const char* getName() const { return name_; }

private:
    const char* name_;
    uint8_t pin_;
    unsigned long debounceDelayMs_;

    bool lastReading_ = false;   // raw reading from the previous update() call
    bool currentState_ = false;  // debounced pin state
    unsigned long lastDebounceTime_ = 0;

    bool rawState_ = false;
    bool detected_ = false;
    unsigned long detectionCount_ = 0;
    unsigned long lastDetectionTime_ = 0;
};

#endif /* INDUCTIVE_SENSOR_H_ */
