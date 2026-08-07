//************************************
//         sensors.h
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

// Read ultrasonic value
void read_ultrasonic(/* Parameters */);

// Read infrared value
void read_infrared(/* Parameters */);

// Bring up the colour sensor. Call once from setup().
void sensors_colour_init(void);

void read_colour(/* Parameters */);

// Print the colour sensor's latest reading to Serial. For debugging.
void colour_print(void);

// Pass in data and average the lot
void sensor_average(/* Parameters */);

#endif /* SENSORS_H_ */
