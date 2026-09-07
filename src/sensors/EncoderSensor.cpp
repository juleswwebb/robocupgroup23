#include "sensors/EncoderSensor.h"
#include <Arduino.h>

EncoderSensor* EncoderSensor::instances_[ENCODER_MAX_INSTANCES] = {nullptr, nullptr, nullptr, nullptr};
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
    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);

    switch (isrSlot_) {
        case 0: attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline0, CHANGE); break;
        case 1: attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline1, CHANGE); break;
        case 2: attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline2, CHANGE); break;
        case 3: attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline3, CHANGE); break;
        default: return false;
    }
    return true;
}

void EncoderSensor::resetPosition() {
    // position_ is written by the ISR, so pause interrupts for the write
    // rather than risk tearing a partially-updated long.
    noInterrupts();
    position_ = 0;
    interrupts();
}

void EncoderSensor::handleChannelAEdge() {
    // Test transition
    aSet_ = digitalRead(pinA_) == HIGH;
    // and adjust counter + if A leads B
    position_ += (aSet_ != bSet_) ? +1 : -1;

    bSet_ = digitalRead(pinB_) == HIGH;
    // and adjust counter + if B follows A
    position_ += (aSet_ == bSet_) ? +1 : -1;
}

void EncoderSensor::isrTrampoline0() { if (instances_[0]) instances_[0]->handleChannelAEdge(); }
void EncoderSensor::isrTrampoline1() { if (instances_[1]) instances_[1]->handleChannelAEdge(); }
void EncoderSensor::isrTrampoline2() { if (instances_[2]) instances_[2]->handleChannelAEdge(); }
void EncoderSensor::isrTrampoline3() { if (instances_[3]) instances_[3]->handleChannelAEdge(); }
