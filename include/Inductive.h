//************************************
//         Inductive.h
//************************************
//
// Task-scheduler-facing entry points for the inductive proximity sensor.
// Owns the actual InductiveSensor instance internally (see Inductive.cpp).

#ifndef INDUCTIVE_H_
#define INDUCTIVE_H_

// Bring up the pin. Call once from setup().
void inductive_init();

// Poll the sensor once. Call periodically from a scheduled task.
void inductive_update();

// Print the detected state (and raw pin level) to Serial. For debugging.
void inductive_print();

#endif /* INDUCTIVE_H_ */
