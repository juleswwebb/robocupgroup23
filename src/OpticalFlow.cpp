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
