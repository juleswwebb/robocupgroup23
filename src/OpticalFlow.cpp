#include "OpticalFlow.h"
#include "sensor_config.h"
#include "sensors/OpticalFlowSensor.h"
#include <Arduino.h>
#include <SPI.h>

static OpticalFlowSensor flowSensor("optical_flow", OPTICAL_FLOW_CS_PIN);

void optical_flow_init() {
    SPI.begin();
    flowSensor.begin();
}

void optical_flow_update() {
    flowSensor.update();
}

int16_t optical_flow_get_delta_x() { return flowSensor.getDeltaX(); }
int16_t optical_flow_get_delta_y() { return flowSensor.getDeltaY(); }
int32_t optical_flow_get_total_x() { return flowSensor.getTotalX(); }
int32_t optical_flow_get_total_y() { return flowSensor.getTotalY(); }
bool optical_flow_is_valid() { return flowSensor.isValid(); }

void optical_flow_print() {
    Serial.print("optical_flow: ");
    if (flowSensor.isValid()) {
        Serial.print("dX=");
        Serial.print(flowSensor.getDeltaX());
        Serial.print(" dY=");
        Serial.print(flowSensor.getDeltaY());
        Serial.print("  totalX=");
        Serial.print(flowSensor.getTotalX());
        Serial.print(" totalY=");
        Serial.println(flowSensor.getTotalY());
    } else {
        Serial.println("invalid");
    }
}
