#include "IMU.h"
#include "sensor_config.h"
#include "sensors/ImuSensor.h"
#include <Arduino.h>

static ImuSensor imuSensor("imu_0", IMU_I2C_ADDRESS, &IMU_I2C_BUS);

void imu_init() {
    imuSensor.begin();
}

void imu_update() {
    imuSensor.update();
}

void imu_print() {
    Serial.print("imu_0: ");
    if (imuSensor.isValid()) {
        Serial.print("heading=");
        Serial.print(imuSensor.getHeading());
        Serial.print(" roll=");
        Serial.print(imuSensor.getRoll());
        Serial.print(" pitch=");
        Serial.print(imuSensor.getPitch());
        Serial.print("  cal(sys/gyro/accel/mag)=");
        Serial.print(imuSensor.getSystemCalibration());
        Serial.print("/");
        Serial.print(imuSensor.getGyroCalibration());
        Serial.print("/");
        Serial.print(imuSensor.getAccelCalibration());
        Serial.print("/");
        Serial.println(imuSensor.getMagCalibration());
    } else {
        Serial.println("invalid");
    }
}
