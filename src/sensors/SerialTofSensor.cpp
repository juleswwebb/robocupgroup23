#include "sensors/SerialTofSensor.h"

SerialTofSensor::SerialTofSensor(const char* name, HardwareSerial& port, uint32_t baud)
    : name_(name), port_(port), baud_(baud) {}

bool SerialTofSensor::begin() {
    port_.begin(baud_);
    return true;
}

void SerialTofSensor::update() {
    while (port_.available()) {
        rx_[rxIndex_] = port_.read();

        if (rx_[0] != 0x59) {
            rxIndex_ = 0;
        } else if (rxIndex_ == 1 && rx_[1] != 0x59) {
            rxIndex_ = 0;
        } else if (rxIndex_ == 8) {
            uint16_t checksum = 0;
            for (uint8_t j = 0; j < 8; j++) {
                checksum += rx_[j];
            }
            if (rx_[8] == (checksum % 256)) {
                lastRangeMM_ = (rx_[2] + rx_[3] * 256) * 10; // cm -> mm
                lastStrength_ = rx_[4] + rx_[5] * 256;
                frameReceived_ = true;
            }
            rxIndex_ = 0;
        } else {
            rxIndex_++;
        }
    }
}
