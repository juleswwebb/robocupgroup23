#include <Arduino.h>

// Deliberately mirrors Group 7's known-working Serial1 setup while removing
// every unrelated robot subsystem. This is a diagnostic firmware, not the
// competition application.
static uint8_t rxBuffer[2048] = {};
static uint8_t txBuffer[8192] = {};
static uint32_t receivedBytes = 0;
static uint32_t lastReportMs = 0;
static uint32_t lastBeaconMs = 0;

void setup() {
    Serial.begin(115200);
    Serial1.addMemoryForRead(rxBuffer, sizeof(rxBuffer));
    Serial1.addMemoryForWrite(txBuffer, sizeof(txBuffer));
    Serial1.begin(115200);

    pinMode(49, OUTPUT);
    digitalWrite(49, HIGH);
    delay(100);

    Serial.println("Serial1 CH9143 isolation test ready");
    Serial1.print("{\"type\":\"probe_ready\"}\n");
}

void loop() {
    while (Serial1.available() > 0) {
        const int value = Serial1.read();
        if (value < 0) break;
        receivedBytes++;
        Serial.write(static_cast<uint8_t>(value));
        Serial1.write(static_cast<uint8_t>(value));
    }

    const uint32_t now = millis();
    if (now - lastReportMs >= 1000) {
        lastReportMs = now;
        Serial.print("Serial1 RX bytes: ");
        Serial.println(receivedBytes);
    }
    if (now - lastBeaconMs >= 1000) {
        lastBeaconMs = now;
        Serial1.print("{\"type\":\"probe_beacon\"}\n");
    }
}
