#include "Encoders.h"
#include "sensor_config.h"
#include "sensors/EncoderSensor.h"
#include <Arduino.h>

static EncoderSensor encoder0("encoder_0", ENCODER0_PIN_A, ENCODER0_PIN_B);
static EncoderSensor encoder1("encoder_1", ENCODER1_PIN_A, ENCODER1_PIN_B);

void encoders_init() {
    encoder0.begin();
    encoder1.begin();
}

void encoders_print() {
    Serial.print("encoder_0: ");
    Serial.print(encoder0.getPosition());
    Serial.print("   encoder_1: ");
    Serial.println(encoder1.getPosition());
}
