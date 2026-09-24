#include "sensors/UltrasonicSensor.h"
#include <Arduino.h>

UltrasonicSensor* UltrasonicSensor::instances_[ULTRASONIC_MAX_INSTANCES] = {nullptr, nullptr, nullptr, nullptr};
uint8_t UltrasonicSensor::instanceCount_ = 0;
uint32_t UltrasonicSensor::lastAnyPingUs_ = 0;
bool UltrasonicSensor::anyPingStarted_ = false;
UltrasonicSensor* UltrasonicSensor::activeSensor_ = nullptr;
uint8_t UltrasonicSensor::nextSlot_ = 0;

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
    lastTriggerMs_ = millis() - pingIntervalMs_;
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
    const uint32_t nowUs = micros();
    if (waiting_) {
        bool captured;
        uint32_t duration;
        noInterrupts();
        captured = captureReady_;
        duration = capturedUs_;
        captureReady_ = false;
        interrupts();
        if (captured) {
            const uint32_t mm = (duration * 343UL + 1000UL) / 2000UL;
            lastValid_ = duration > 0 && duration <= 38000UL && mm >= 20 && mm <= 5000;
            if (lastValid_) lastDistanceMM_ = static_cast<uint16_t>(mm);
            waiting_ = false;
            activeSensor_ = nullptr;
        } else if (nowUs - pingStartedUs_ >= 38000UL) {
            noInterrupts();
            captureArmed_ = false;
            sawRise_ = false;
            interrupts();
            lastValid_ = false;
            waiting_ = false;
            activeSensor_ = nullptr;
        }
        return;
    }
    // One shared acoustic slot: at least 60 ms from the previous trigger,
    // and never while another HC-SR04 still awaits its echo.
    if (activeSensor_ || isrSlot_ != nextSlot_ ||
        (anyPingStarted_ && nowUs - lastAnyPingUs_ < 60000UL) ||
        millis() - lastTriggerMs_ < pingIntervalMs_) return;
    noInterrupts();
    captureReady_ = false;
    sawRise_ = false;
    captureArmed_ = true;
    interrupts();
    activeSensor_ = this;
    nextSlot_ = (nextSlot_ + 1) % instanceCount_;
    lastAnyPingUs_ = nowUs;
    anyPingStarted_ = true;
    pingStartedUs_ = nowUs;
    lastTriggerMs_ = millis();
    waiting_ = true;
    triggerPing();
}

void UltrasonicSensor::handleEchoEdge() {
    if (!captureArmed_) return;
    if (digitalRead(echoPin_) == HIGH) {
        echoStartUs_ = micros();
        sawRise_ = true;
        return;
    }
    if (!sawRise_) return;
    capturedUs_ = micros() - echoStartUs_;
    captureReady_ = true;
    captureArmed_ = false;
}

void UltrasonicSensor::isrTrampoline0() { if (instances_[0]) instances_[0]->handleEchoEdge(); }
void UltrasonicSensor::isrTrampoline1() { if (instances_[1]) instances_[1]->handleEchoEdge(); }
void UltrasonicSensor::isrTrampoline2() { if (instances_[2]) instances_[2]->handleEchoEdge(); }
void UltrasonicSensor::isrTrampoline3() { if (instances_[3]) instances_[3]->handleEchoEdge(); }
