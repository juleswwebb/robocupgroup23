//************************************
//         OpticalFlow.h
//************************************
//
// Task-scheduler-facing entry points for the optical flow sensor. Owns the
// actual OpticalFlowSensor instance internally (see OpticalFlow.cpp).

#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

#include <stdint.h>

// Bring up SPI and the sensor. Call once from setup().
void optical_flow_init();

// Poll the sensor once. Call periodically from a scheduled task.
void optical_flow_update();

// Print the latest delta + running total to Serial. For debugging.
void optical_flow_print();

// Motion since the last update (sensor counts, not mm).
int16_t optical_flow_get_delta_x();
int16_t optical_flow_get_delta_y();

// Running sum of those deltas - a dead-reckoning estimate, so it drifts.
int32_t optical_flow_get_total_x();
int32_t optical_flow_get_total_y();

bool optical_flow_is_valid();

#endif /* OPTICAL_FLOW_H_ */
