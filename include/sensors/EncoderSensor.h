//************************************
//         EncoderSensor.h
//************************************
//
// Quadrature encoder with a full 4x state-table decoder. Both channels use
// CHANGE interrupts: each legal A/B state transition adds one signed count.
// Same-state bounce and impossible diagonal jumps are ignored.

#ifndef ENCODER_SENSOR_H_
#define ENCODER_SENSOR_H_

#include <stdint.h>

#define ENCODER_MAX_INSTANCES 4

class EncoderSensor {
public:
    EncoderSensor(const char* name, uint8_t pinA, uint8_t pinB);

    bool begin();

    long getPosition() const;
    void resetPosition();
    const char* getName() const { return name_; }
    bool getChannelA() const;
    bool getChannelB() const;
    unsigned long getTransitionCount() const;

private:
    void handleEdge(); // called from either ISR trampoline for this instance

    const char* name_;
    uint8_t pinA_;
    uint8_t pinB_;
    volatile long position_ = 0;
    volatile uint8_t lastState_ = 0;
    volatile unsigned long transitionCount_ = 0;

    // attachInterrupt() needs a plain function pointer, so each live
    // instance is assigned one of a small fixed set of trampoline
    // functions, which then dispatch back into the right instance.
    uint8_t isrSlot_;
    static EncoderSensor* instances_[ENCODER_MAX_INSTANCES];
    static uint8_t instanceCount_;
    static void isrTrampoline0A();
    static void isrTrampoline0B();
    static void isrTrampoline1A();
    static void isrTrampoline1B();
    static void isrTrampoline2A();
    static void isrTrampoline2B();
    static void isrTrampoline3A();
    static void isrTrampoline3B();
};

#endif /* ENCODER_SENSOR_H_ */
