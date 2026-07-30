//************************************
//         OpticalFlowSensor.h
//************************************
//
// Wraps the Bitcraze PMW3901 optical flow sensor (SPI). Each update()
// reports the relative motion (in sensor "counts", not mm) since the last
// call - it's a dead-reckoning input, not a distance sensor, so it doesn't
// implement the DistanceSensor interface. getTotalX()/getTotalY() keep a
// running sum of those deltas as a simple position estimate; expect it to
// drift over time like any dead-reckoning sensor.

#ifndef OPTICAL_FLOW_SENSOR_H_
#define OPTICAL_FLOW_SENSOR_H_

#include <stdint.h>
#include <Bitcraze_PMW3901.h>

class OpticalFlowSensor {
public:
    OpticalFlowSensor(const char* name, uint8_t csPin);

    bool begin();
    void update(); // reads the motion count accumulated since the last call

    int16_t getDeltaX() const { return deltaX_; }
    int16_t getDeltaY() const { return deltaY_; }
    int32_t getTotalX() const { return totalX_; }
    int32_t getTotalY() const { return totalY_; }
    bool isValid() const { return initialized_; }
    const char* getName() const { return name_; }

private:
    const char* name_;
    Bitcraze_PMW3901 sensor_;
    int16_t deltaX_ = 0;
    int16_t deltaY_ = 0;
    int32_t totalX_ = 0;
    int32_t totalY_ = 0;
    bool initialized_ = false;
};

#endif /* OPTICAL_FLOW_SENSOR_H_ */
