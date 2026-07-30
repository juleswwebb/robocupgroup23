#include "sensors/UltrasonicSensor.h"
#include <Arduino.h>

UltrasonicSensor* UltrasonicSensor::instances_[ULTRASONIC_MAX_INSTANCES] = {nullptr, nullptr, nullptr, nullptr};
uint8_t UltrasonicSensor::instanceCount_ = 0;

UltrasonicSensor::UltrasonicSensor(const char* name, uint8_t trigPin, uint8_t echoPin,
                                    uint32_t pingIntervalMs)
    : name_(name), trigPin_(trigPin), echoPin_(echoPin), pingIntervalMs_(pingIntervalMs) {
    if (instanceCount_ < ULTRASONIC_MAX_INSTANCES) {
        isrSlot_ = instanceCount_;
        instances_[instanceCount_++] = this;
    } else {
        Serial.print("UltrasonicSensor '");
        Serial.print(name_);
        Serial.println("': too many instances (max 4), echo interrupt won't be wired up");
        isrSlot_ = 0xFF;
    }
}

bool UltrasonicSensor::begin() {
    pinMode(trigPin_, OUTPUT);
    digitalWrite(trigPin_, LOW);
    pinMode(echoPin_, INPUT);

    switch (isrSlot_) {
        case 0: attachInterrupt(digitalPinToInterrupt(echoPin_), isrTrampoline0, CHANGE); break;
        case 1: attachInterrupt(digitalPinToInterrupt(echoPin_), isrTrampoline1, CHANGE); break;
        case 2: attachInterrupt(digitalPinToInterrupt(echoPin_), isrTrampoline2, CHANGE); break;
        case 3: attachInterrupt(digitalPinToInterrupt(echoPin_), isrTrampoline3, CHANGE); break;
        default: return false;
    }
    return true;
}

void UltrasonicSensor::triggerPing() {
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);
}

void UltrasonicSensor::update() {
    unsigned long now = millis();
    if (now - lastTriggerMs_ >= pingIntervalMs_) {
        lastTriggerMs_ = now;
        triggerPing();
    }
}

void UltrasonicSensor::handleEchoEdge() {
    if (digitalRead(echoPin_) == HIGH) {
        echoStartUs_ = micros();
        return;
    }

    uint32_t duration = micros() - echoStartUs_;
    if (duration == 0 || duration > 38000UL) { // ~38ms is this sensor family's own no-echo timeout
        lastValid_ = false;
        return;
    }

    // speed of sound ~343 m/s = 0.343 mm/us round trip -> 0.1715 mm/us one-way
    lastDistanceMM_ = (uint16_t)(duration * 343UL / 2000UL);
    lastValid_ = true;
}

void UltrasonicSensor::isrTrampoline0() { if (instances_[0]) instances_[0]->handleEchoEdge(); }
void UltrasonicSensor::isrTrampoline1() { if (instances_[1]) instances_[1]->handleEchoEdge(); }
void UltrasonicSensor::isrTrampoline2() { if (instances_[2]) instances_[2]->handleEchoEdge(); }
void UltrasonicSensor::isrTrampoline3() { if (instances_[3]) instances_[3]->handleEchoEdge(); }
