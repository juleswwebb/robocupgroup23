//************************************
//         ColourSensor.h
//************************************
//
// Wraps the Adafruit TCS34725 RGBC colour sensor.
//
// Note: getRawData() blocks for the sensor's integration time while it
// waits for a measurement to complete (50ms by default below) - fine to
// call occasionally from a scheduled task, but don't poll this faster than
// that interval or you'll stall the scheduler waiting on it.

#ifndef COLOUR_SENSOR_H_
#define COLOUR_SENSOR_H_

#include <stdint.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

class ColourSensor {
public:
    // address/bus: where this sensor is wired. integrationTime/gain: passed
    // straight through to the Adafruit library (see its TCS34725_INTEGRATIONTIME_*
    // / TCS34725_GAIN_* constants).
    ColourSensor(const char* name, uint8_t address = TCS34725_ADDRESS, TwoWire* bus = &Wire,
                 uint8_t integrationTime = TCS34725_INTEGRATIONTIME_50MS,
                 tcs34725Gain_t gain = TCS34725_GAIN_4X);

    bool begin();
    void update(); // blocking read - see class comment above

    uint16_t getRed() const { return red_; }
    uint16_t getGreen() const { return green_; }
    uint16_t getBlue() const { return blue_; }
    uint16_t getClear() const { return clear_; }
    bool isValid() const { return initialized_; }
    const char* getName() const { return name_; }

private:
    const char* name_;
    uint8_t address_;
    TwoWire* bus_;
    Adafruit_TCS34725 sensor_;
    uint16_t red_ = 0;
    uint16_t green_ = 0;
    uint16_t blue_ = 0;
    uint16_t clear_ = 0;
    bool initialized_ = false;
};

#endif /* COLOUR_SENSOR_H_ */
