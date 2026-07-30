//************************************
//         OpticalFlow.h
//************************************
//
// Task-scheduler-facing entry points for the optical flow sensor. Owns the
// actual OpticalFlowSensor instance internally (see OpticalFlow.cpp).

#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

// Bring up SPI and the sensor. Call once from setup().
void optical_flow_init();

// Poll the sensor once. Call periodically from a scheduled task.
void optical_flow_update();

// Print the latest delta + running total to Serial. For debugging.
void optical_flow_print();

#endif /* OPTICAL_FLOW_H_ */
