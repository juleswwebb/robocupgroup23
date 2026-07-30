#include "Inductive.h"
#include "sensor_config.h"
#include "sensors/InductiveSensor.h"
#include <Arduino.h>

static InductiveSensor inductiveSensor("inductive_0", INDUCTIVE_PIN);

void inductive_init() {
    inductiveSensor.begin();
}

void inductive_update() {
    inductiveSensor.update();
}

void inductive_print() {
    Serial.print("inductive_0: detected=");
    Serial.print(inductiveSensor.isDetected() ? "true" : "false");
    Serial.print(" count=");
    Serial.print(inductiveSensor.getDetectionCount());
    Serial.print(" (raw pin=");
    Serial.print(inductiveSensor.getRawPinState());
    Serial.println(")");
}
