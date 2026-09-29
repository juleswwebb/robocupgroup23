#include "MagnetControl.h"
#include "sensor_config.h"

#include <Arduino.h>

namespace {
bool magnetOn = false;
unsigned long lastCommandMs = 0;
}

void magnet_control_init() {
    pinMode(MAGNET_PIN, OUTPUT);
    magnet_control_off();
}

void magnet_control_set(bool enabled) {
    magnetOn = enabled;
    digitalWrite(MAGNET_PIN, magnetOn ? HIGH : LOW);
    lastCommandMs = millis();
}

void magnet_control_off() {
    magnet_control_set(false);
}

void magnet_control_update() {
    if (magnetOn && millis() - lastCommandMs > MAGNET_COMMAND_TIMEOUT_MS) {
        magnet_control_off();
    }
}

bool magnet_control_is_on() {
    return magnetOn;
}
