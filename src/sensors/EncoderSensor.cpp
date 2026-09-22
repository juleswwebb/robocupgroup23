#include "sensors/EncoderSensor.h"
#include "sensor_config.h"
#include <Arduino.h>

EncoderSensor* EncoderSensor::instances_[ENCODER_MAX_INSTANCES] = {
    nullptr, nullptr, nullptr, nullptr
};
uint8_t EncoderSensor::instanceCount_ = 0;

EncoderSensor::EncoderSensor(const char* name, uint8_t pinA, uint8_t pinB)
    : name_(name), pinA_(pinA), pinB_(pinB) {
    if (instanceCount_ < ENCODER_MAX_INSTANCES) {
        isrSlot_ = instanceCount_;
        instances_[instanceCount_++] = this;
    } else {
        Serial.print("EncoderSensor '");
        Serial.print(name_);
        Serial.println("': too many instances (max 4), interrupt won't be wired up");
        isrSlot_ = 0xFF;
    }
}

bool EncoderSensor::begin() {
#if ENCODER_USE_INTERNAL_PULLUPS
    pinMode(pinA_, INPUT_PULLUP);
    pinMode(pinB_, INPUT_PULLUP);
#else
    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);
#endif

    // Establish the initial AB state before interrupts start.
    lastState_ = (digitalRead(pinA_) ? 0x02 : 0x00)
               | (digitalRead(pinB_) ? 0x01 : 0x00);

    switch (isrSlot_) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline0A, CHANGE);
            attachInterrupt(digitalPinToInterrupt(pinB_), isrTrampoline0B, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline1A, CHANGE);
            attachInterrupt(digitalPinToInterrupt(pinB_), isrTrampoline1B, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline2A, CHANGE);
            attachInterrupt(digitalPinToInterrupt(pinB_), isrTrampoline2B, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline3A, CHANGE);
            attachInterrupt(digitalPinToInterrupt(pinB_), isrTrampoline3B, CHANGE);
            break;
        default:
            return false;
    }
    return true;
}

long EncoderSensor::getPosition() const {
    noInterrupts();
    const long value = position_;
    interrupts();
    return value;
}

bool EncoderSensor::getChannelA() const {
    return digitalRead(pinA_) == HIGH;
}

bool EncoderSensor::getChannelB() const {
    return digitalRead(pinB_) == HIGH;
}

unsigned long EncoderSensor::getTransitionCount() const {
    noInterrupts();
    const unsigned long value = transitionCount_;
    interrupts();
    return value;
}

void EncoderSensor::resetPosition() {
    noInterrupts();
    position_ = 0;
    transitionCount_ = 0;
    lastState_ = (digitalRead(pinA_) ? 0x02 : 0x00)
               | (digitalRead(pinB_) ? 0x01 : 0x00);
    interrupts();
}

void EncoderSensor::handleEdge() {
    // Index = old AB state (bits 3..2) followed by new AB state (bits 1..0).
    // Legal one-bit quadrature changes contribute +/-1; invalid/bounce steps
    // contribute zero rather than corrupting the count.
    static const int8_t transitionTable[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    const uint8_t currentState = (digitalRead(pinA_) ? 0x02 : 0x00)
                               | (digitalRead(pinB_) ? 0x01 : 0x00);
    const int8_t change = transitionTable[(lastState_ << 2) | currentState];
    if (change != 0) {
        position_ += change;
        transitionCount_++;
    }
    lastState_ = currentState;
}

void EncoderSensor::isrTrampoline0A() { if (instances_[0]) instances_[0]->handleEdge(); }
void EncoderSensor::isrTrampoline0B() { if (instances_[0]) instances_[0]->handleEdge(); }
void EncoderSensor::isrTrampoline1A() { if (instances_[1]) instances_[1]->handleEdge(); }
void EncoderSensor::isrTrampoline1B() { if (instances_[1]) instances_[1]->handleEdge(); }
void EncoderSensor::isrTrampoline2A() { if (instances_[2]) instances_[2]->handleEdge(); }
void EncoderSensor::isrTrampoline2B() { if (instances_[2]) instances_[2]->handleEdge(); }
void EncoderSensor::isrTrampoline3A() { if (instances_[3]) instances_[3]->handleEdge(); }
void EncoderSensor::isrTrampoline3B() { if (instances_[3]) instances_[3]->handleEdge(); }
