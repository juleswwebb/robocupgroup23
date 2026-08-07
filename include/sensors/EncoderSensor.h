//************************************
//         EncoderSensor.h
//************************************
//
// Quadrature encoder, interrupt-driven on channel A only (checks B's
// current state to determine direction on each A edge) - same simplified
// quadrature decode as the reference sketch this was ported from. This
// gives 2x counts per detent instead of a full 4x decode (which would
// interrupt on both channels), but that's enough for basic dead-reckoning
// and matches what's already proven working.

#ifndef ENCODER_SENSOR_H_
#define ENCODER_SENSOR_H_

#include <stdint.h>

#define ENCODER_MAX_INSTANCES 4

class EncoderSensor {
public:
    EncoderSensor(const char* name, uint8_t pinA, uint8_t pinB);

    bool begin();

    long getPosition() const { return position_; }
    const char* getName() const { return name_; }

private:
    void handleChannelAEdge(); // called from the ISR trampoline for this instance's slot

    const char* name_;
    uint8_t pinA_;
    uint8_t pinB_;
    volatile long position_ = 0;
    volatile bool aSet_ = false;
    volatile bool bSet_ = false;

    // attachInterrupt() needs a plain function pointer, so each live
    // instance is assigned one of a small fixed set of trampoline
    // functions, which then dispatch back into the right instance.
    uint8_t isrSlot_;
    static EncoderSensor* instances_[ENCODER_MAX_INSTANCES];
    static uint8_t instanceCount_;
    static void isrTrampoline0();
    static void isrTrampoline1();
    static void isrTrampoline2();
    static void isrTrampoline3();
};

#endif /* ENCODER_SENSOR_H_ */
