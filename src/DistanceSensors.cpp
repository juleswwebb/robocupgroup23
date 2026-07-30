#include "DistanceSensors.h"
#include "sensor_config.h"
#include "sensors/VL53L0XSensor.h"
#include "sensors/VL53L1XSensor.h"
#include "sensors/MatrixLidarSensor.h"
#include "sensors/SerialTofSensor.h"
#include "sensors/SensorManager.h"
#include <Arduino.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

// All 7 VL53 XSHUT lines are wired through this single SX1509 expander.
static SX1509 xshutExpander;

// Registered in physical XSHUT wiring order (0..6) - this is also the
// order they're brought out of reset and addressed in, see sensor_config.h.
static VL53L0XSensor tofXshut0("tof_xshut0", &xshutExpander, XSHUT0_PIN, VL53L0X_ADDR_BASE + 0, &VL53_I2C_BUS);
static VL53L0XSensor tofXshut1("tof_xshut1", &xshutExpander, XSHUT1_PIN, VL53L0X_ADDR_BASE + 1, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut2("tof_xshut2", &xshutExpander, XSHUT2_PIN, VL53L1X_ADDR_BASE + 0, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut3("tof_xshut3", &xshutExpander, XSHUT3_PIN, VL53L1X_ADDR_BASE + 1, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut4("tof_xshut4", &xshutExpander, XSHUT4_PIN, VL53L1X_ADDR_BASE + 2, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut5("tof_xshut5", &xshutExpander, XSHUT5_PIN, VL53L1X_ADDR_BASE + 3, &VL53_I2C_BUS);
static VL53L0XSensor tofXshut6("tof_xshut6", &xshutExpander, XSHUT6_PIN, VL53L0X_ADDR_BASE + 2, &VL53_I2C_BUS);

static MatrixLidarSensor tof8x8("tof_8x8", MATRIX_LIDAR_ADDR, &MATRIX_LIDAR_I2C_BUS);
static SerialTofSensor tofSerial("tof_serial", Serial2, SERIAL_TOF_BAUD);

static SensorManager sensorManager;

void distance_sensors_init() {
    VL53_I2C_BUS.begin();
    VL53_I2C_BUS.setClock(400000); // 400 kHz I2C, matches the vendor examples
    MATRIX_LIDAR_I2C_BUS.begin();
    MATRIX_LIDAR_I2C_BUS.setClock(100000); // conservative default for this module

    if (xshutExpander.begin(SX1509_I2C_ADDRESS, SX1509_I2C_BUS) == 0) {
        Serial.println("distance_sensors_init: SX1509 expander failed to initialise");
    }

    sensorManager.addSensor(&tofXshut0);
    sensorManager.addSensor(&tofXshut1);
    sensorManager.addSensor(&tofXshut2);
    sensorManager.addSensor(&tofXshut3);
    sensorManager.addSensor(&tofXshut4);
    sensorManager.addSensor(&tofXshut5);
    sensorManager.addSensor(&tofXshut6);
    sensorManager.addSensor(&tof8x8);
    sensorManager.addSensor(&tofSerial);

    if (!sensorManager.beginAll()) {
        Serial.println("distance_sensors_init: one or more sensors failed to initialise");
    }
}

void distance_sensors_update() {
    sensorManager.updateAll();
}

void distance_sensors_print() {
    for (uint8_t i = 0; i < sensorManager.count(); i++) {
        DistanceSensor* s = sensorManager.get(i);
        Serial.print(s->getName());
        Serial.print(": ");
        if (s->isValid()) {
            Serial.print(s->getDistanceMM());
            Serial.print(" mm");
        } else {
            Serial.print("invalid");
        }
        Serial.print("   ");
    }
    Serial.println();
}

DistanceSensor* distance_sensor_get(const char* name) {
    return sensorManager.get(name);
}
