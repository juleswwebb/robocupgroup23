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

#endif /* ENCODERS_H_ */
