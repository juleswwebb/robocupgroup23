//************************************
//         Encoders.h
//************************************
//
// Task-scheduler-facing entry points for the two quadrature encoders.
// Owns the actual EncoderSensor instances internally (see Encoders.cpp).
// There's no update() - position is kept current by interrupts in the
// background, so there's nothing to poll; just print() to read it.

#ifndef ENCODERS_H_
#define ENCODERS_H_

// Bring up the pins + interrupts. Call once from setup().
void encoders_init();

// Print both encoders' current position to Serial. For debugging.
void encoders_print();

// Current count for encoder 0 or 1. Returns 0 for any other index.
long encoder_get_position(unsigned char index);

// Live input diagnostics. If A/B never change and valid edges stay at zero,
// the fault is electrical (supply, ground, connector or pin path), not math.
bool encoder_get_channel_a(unsigned char index);
bool encoder_get_channel_b(unsigned char index);
unsigned long encoder_get_transition_count(unsigned char index);

// Zero both counts - useful before a measured drive test.
void encoders_reset();

#endif /* ENCODERS_H_ */
