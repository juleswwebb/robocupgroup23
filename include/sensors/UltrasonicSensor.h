//************************************
//         UltrasonicSensor.h
//************************************
//
// HC-SR04-style ultrasonic distance sensor (separate trig/echo pins).
// Interrupt-driven rather than using the classic blocking pulseIn(): update()
// just checks whether it's time to fire the next ping. All instances share
// a ping slot so adjacent sensors cannot hear one another's trigger, and an
// ISR on the echo pin captures the actual pulse width whenever it arrives.
// getDistanceMM()/isValid() just return the last value the ISR captured.

#ifndef ULTRASONIC_SENSOR_H_
#define ULTRASONIC_SENSOR_H_

#include <stdint.h>
#include "sensors/DistanceSensor.h"

#define ULTRASONIC_MAX_INSTANCES 4

class UltrasonicSensor : public DistanceSensor {
public:
    UltrasonicSensor(const char* name, uint8_t trigPin, uint8_t echoPin,
                      uint32_t pingIntervalMs = 60);

    bool begin() override;
    void update() override;
    uint16_t getDistanceMM() const override { return lastDistanceMM_; }
    // False if no echo has come back yet, or the last one timed out
    // (nothing in range within the sensor's own ~38ms echo window).
    bool isValid() const override { return lastValid_; }
    const char* getName() const override { return name_; }

private:
    void triggerPing();
    void handleEchoEdge(); // called from the ISR trampoline for this instance's slot

    const char* name_;
    uint8_t trigPin_;
    uint8_t echoPin_;
    uint32_t pingIntervalMs_;
    unsigned long lastTriggerMs_ = 0;
    volatile uint32_t echoStartUs_ = 0;
    volatile uint16_t lastDistanceMM_ = 0;
    volatile bool lastValid_ = false;
    volatile bool captureArmed_ = false;
    volatile bool sawRise_ = false;
    volatile bool captureReady_ = false;
    volatile uint32_t capturedUs_ = 0;
    bool waiting_ = false;
    uint32_t pingStartedUs_ = 0;
    static uint32_t lastAnyPingUs_;
    static bool anyPingStarted_;
    static UltrasonicSensor* activeSensor_;
    static uint8_t nextSlot_;

    // attachInterrupt() needs a plain function pointer, so each live
    // instance is assigned one of a small fixed set of trampoline
    // functions, which then dispatch back into the right instance.
    uint8_t isrSlot_;
    static UltrasonicSensor* instances_[ULTRASONIC_MAX_INSTANCES];
    static uint8_t instanceCount_;
    static void isrTrampoline0();
    static void isrTrampoline1();
    static void isrTrampoline2();
    static void isrTrampoline3();
};

#endif /* ULTRASONIC_SENSOR_H_ */
