//************************************
//         sensors.cpp
//************************************

// This file contains functions used to read and average
// the sensors.

#include "sensors.h"
#include "Arduino.h"
#include "sensors/ColourSensor.h"

// Read ultrasonic value
void read_ultrasonic(/* Parameters */) {
  Serial.println("Ultrasonic value \n");
}

// Read infrared value
void read_infrared(/* Parameters */) {
  Serial.println("Infrared value \n");
}

// TCS34725 on I2C bus 1 (Wire1), address 0x29.
static ColourSensor colourSensor("colour_0", TCS34725_ADDRESS, &Wire1);

void sensors_colour_init(void) {
  colourSensor.begin();
}

// Read colour sensor value
void read_colour(/* Parameters */) {
  colourSensor.update();
}

void colour_print(void) {
  Serial.print("colour_0: R=");
  Serial.print(colourSensor.getRed());
  Serial.print(" G=");
  Serial.print(colourSensor.getGreen());
  Serial.print(" B=");
  Serial.print(colourSensor.getBlue());
  Serial.print(" C=");
  Serial.println(colourSensor.getClear());
}

uint16_t colour_get_red(void) { return colourSensor.getRed(); }
uint16_t colour_get_green(void) { return colourSensor.getGreen(); }
uint16_t colour_get_blue(void) { return colourSensor.getBlue(); }
uint16_t colour_get_clear(void) { return colourSensor.getClear(); }
bool colour_is_valid(void) { return colourSensor.isValid(); }

// Pass in data and average the lot
void sensor_average(/* Parameters */) {
  Serial.println("Averaging the sensors \n");
}
