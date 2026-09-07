//************************************
//         sensors.h
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>

// Read ultrasonic value
void read_ultrasonic(/* Parameters */);

// Read infrared value
void read_infrared(/* Parameters */);

// Bring up the colour sensor. Call once from setup().
void sensors_colour_init(void);

void read_colour(/* Parameters */);

// Print the colour sensor's latest reading to Serial. For debugging.
void colour_print(void);

// Latest raw RGBC readings. Only meaningful when colour_is_valid().
uint16_t colour_get_red(void);
uint16_t colour_get_green(void);
uint16_t colour_get_blue(void);
uint16_t colour_get_clear(void);
bool colour_is_valid(void);

// Pass in data and average the lot
void sensor_average(/* Parameters */);

#endif /* SENSORS_H_ */
