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

long encoder_get_position(unsigned char index) {
    if (index == 0) return encoder0.getPosition();
    if (index == 1) return encoder1.getPosition();
    return 0;
}

bool encoder_get_channel_a(unsigned char index) {
    if (index == 0) return encoder0.getChannelA();
    if (index == 1) return encoder1.getChannelA();
    return false;
}

bool encoder_get_channel_b(unsigned char index) {
    if (index == 0) return encoder0.getChannelB();
    if (index == 1) return encoder1.getChannelB();
    return false;
}

unsigned long encoder_get_transition_count(unsigned char index) {
    if (index == 0) return encoder0.getTransitionCount();
    if (index == 1) return encoder1.getTransitionCount();
    return 0;
}

void encoders_reset() {
    encoder0.resetPosition();
    encoder1.resetPosition();
}

void encoders_print() {
    Serial.print("encoder_0: ");
    Serial.print(encoder0.getPosition());
    Serial.print(" [A=");
    Serial.print(encoder0.getChannelA());
    Serial.print(" B=");
    Serial.print(encoder0.getChannelB());
    Serial.print(" edges=");
    Serial.print(encoder0.getTransitionCount());
    Serial.print("]");
    Serial.print("   encoder_1: ");
    Serial.print(encoder1.getPosition());
    Serial.print(" [A=");
    Serial.print(encoder1.getChannelA());
    Serial.print(" B=");
    Serial.print(encoder1.getChannelB());
    Serial.print(" edges=");
    Serial.print(encoder1.getTransitionCount());
    Serial.println("]");
}
