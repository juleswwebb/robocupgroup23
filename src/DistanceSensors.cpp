#include "DistanceSensors.h"
#include "sensor_config.h"
#include "sensors/VL53L1XSensor.h"
#include "sensors/MatrixLidarSensor.h"
#if SERIAL_TOF_ENABLED
#include "sensors/SerialTofSensor.h"
#endif
#if IR_DISTANCE_SENSORS_ENABLED
#include "sensors/IRDistanceSensor.h"
#endif
#if ULTRASONIC_ENABLED
#include "sensors/UltrasonicSensor.h"
#endif
#include "sensors/SensorManager.h"
#include <Arduino.h>
#include <Wire.h>
#include <SparkFunSX1509.h>

// Six point VL53L1X sensors use the add-on SX1509 at 0x71 on IO5..IO10.
static SX1509 xshutExpander;
static bool xshutExpanderReady = false;

// The two top L0X units are no longer installed. The active L1X channels are
// assigned unique addresses sequentially, regardless of their IO pin order.
static VL53L1XSensor tofXshut3("tof_xshut3", &xshutExpander, TOF_XSHUT3_IO, VL53L1X_ADDR_BASE + 0, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut4("tof_xshut4", &xshutExpander, TOF_XSHUT4_IO, VL53L1X_ADDR_BASE + 1, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut5("tof_xshut5", &xshutExpander, TOF_XSHUT5_IO, VL53L1X_ADDR_BASE + 2, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut6("tof_xshut6", &xshutExpander, TOF_XSHUT6_IO, VL53L1X_ADDR_BASE + 3, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut7("tof_xshut7", &xshutExpander, TOF_XSHUT7_IO, VL53L1X_ADDR_BASE + 4, &VL53_I2C_BUS);
static VL53L1XSensor tofXshut8("tof_xshut8", &xshutExpander, TOF_XSHUT8_IO, VL53L1X_ADDR_BASE + 5, &VL53_I2C_BUS);

static MatrixLidarSensor tof8x8("tof_8x8", MATRIX_LIDAR_ADDR, &MATRIX_LIDAR_I2C_BUS);
#if SERIAL_TOF_ENABLED
static SerialTofSensor tofSerial("tof_serial", SERIAL_TOF_PORT, SERIAL_TOF_BAUD);
#endif

#if IR_DISTANCE_SENSORS_ENABLED
static IRDistanceSensor ir0("ir_0", IR0_PIN);
static IRDistanceSensor ir1("ir_1", IR1_PIN);
static IRDistanceSensor ir2("ir_2", IR2_PIN);
static IRDistanceSensor ir3("ir_3", IR3_PIN);
#endif

#if ULTRASONIC_ENABLED
static UltrasonicSensor ultrasonic0("ultrasonic_0", ULTRASONIC0_TRIG_PIN, ULTRASONIC0_ECHO_PIN);
static UltrasonicSensor ultrasonic1("ultrasonic_1", ULTRASONIC1_TRIG_PIN, ULTRASONIC1_ECHO_PIN);
#endif

static SensorManager sensorManager;

void distance_sensors_init() {
    VL53_I2C_BUS.begin();
    VL53_I2C_BUS.setClock(400000); // 400 kHz I2C, matches the vendor examples
    MATRIX_LIDAR_I2C_BUS.begin();
    MATRIX_LIDAR_I2C_BUS.setClock(100000); // conservative default for this module

    xshutExpanderReady = xshutExpander.begin(SX1509_I2C_ADDRESS, SX1509_I2C_BUS);
    if (!xshutExpanderReady) {
        Serial.println("distance_sensors_init: SX1509 expander failed to initialise");
    }

    // Match the proven bring-up sequence used by the reference robot: make
    // every SX1509 pin an output and hold the entire expander bank low before
    // releasing any VL53 sensor. SensorManager then enables each used channel
    // one at a time and assigns its unique I2C address before proceeding.
    for (uint8_t io = 0; io < 16; ++io) {
        xshutExpander.pinMode(io, OUTPUT);
        xshutExpander.digitalWrite(io, LOW);
    }
    delay(100);

    sensorManager.addSensor(&tofXshut3);
    sensorManager.addSensor(&tofXshut4);
    sensorManager.addSensor(&tofXshut5);
    sensorManager.addSensor(&tofXshut6);
    sensorManager.addSensor(&tofXshut7);
    sensorManager.addSensor(&tofXshut8);
    sensorManager.addSensor(&tof8x8);
#if SERIAL_TOF_ENABLED
    sensorManager.addSensor(&tofSerial);
#endif
#if IR_DISTANCE_SENSORS_ENABLED
    sensorManager.addSensor(&ir0);
    sensorManager.addSensor(&ir1);
    sensorManager.addSensor(&ir2);
    sensorManager.addSensor(&ir3);
#endif
#if ULTRASONIC_ENABLED
    sensorManager.addSensor(&ultrasonic0);
    sensorManager.addSensor(&ultrasonic1);
#endif

    if (!sensorManager.beginAll()) {
        Serial.println("distance_sensors_init: one or more sensors failed to initialise");
    }
}

void distance_sensors_update() {
    sensorManager.updateAll();
}

void distance_sensors_print() {
    unsigned char pointTofIndex = 0;
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
        if (strncmp(s->getName(), "tof_xshut", 9) == 0) {
            PointTofDiagnostic diagnostic;
            if (distance_sensors_get_point_tof_diagnostic(pointTofIndex, &diagnostic)) {
                Serial.print(" ["); Serial.print(diagnostic.model);
                Serial.print(" IO"); Serial.print(diagnostic.xshutIo);
                Serial.print(" init="); Serial.print(diagnostic.initialized ? 1 : 0);
                Serial.print(" sample="); Serial.print(diagnostic.hasSample ? 1 : 0);
                Serial.print(" range_status="); Serial.print(diagnostic.rangeStatus);
                Serial.print(" i2c_status="); Serial.print(diagnostic.i2cStatus);
                Serial.print(" model_id=0x"); Serial.print(diagnostic.modelId, HEX);
                Serial.print(" attempts="); Serial.print(diagnostic.initAttempts);
                Serial.print(']');
            }
            pointTofIndex++;
        }
        Serial.print("   ");
    }
    Serial.println();
}

DistanceSensor* distance_sensor_get(const char* name) {
    return sensorManager.get(name);
}

unsigned char distance_sensors_count() {
    return sensorManager.count();
}

DistanceSensor* distance_sensor_get_by_index(unsigned char index) {
    return sensorManager.get(index);
}

static uint32_t tofSampleAge(bool hasSample, uint32_t lastSampleAtMs) {
    return hasSample ? millis() - lastSampleAtMs : UINT32_MAX;
}

template <typename SensorType>
static bool fillPointTofDiagnostic(const SensorType* sensor, const char* model,
                                   PointTofDiagnostic* diagnostic) {
    if (sensor == nullptr || diagnostic == nullptr) return false;
    diagnostic->name = sensor->getName();
    diagnostic->model = model;
    diagnostic->xshutIo = sensor->getXshutPin();
    diagnostic->initialized = sensor->isInitialized();
    diagnostic->hasSample = sensor->hasSample();
    diagnostic->valid = sensor->isValid();
    diagnostic->noReturn = sensor->isNoReturn();
    diagnostic->rangeStatus = sensor->getRangeStatus();
    diagnostic->i2cStatus = sensor->getI2CStatus();
    diagnostic->initAttempts = sensor->getInitAttempts();
    diagnostic->modelId = sensor->getModelId();
    diagnostic->sampleAgeMs = tofSampleAge(diagnostic->hasSample,
                                           sensor->getLastSampleAtMs());
    return true;
}

unsigned char distance_sensors_point_tof_count() {
    return 6;
}

bool distance_sensors_get_point_tof_diagnostic(unsigned char index,
                                                PointTofDiagnostic* diagnostic) {
    switch (index) {
        case 0: return fillPointTofDiagnostic(&tofXshut3, "VL53L1X", diagnostic);
        case 1: return fillPointTofDiagnostic(&tofXshut4, "VL53L1X", diagnostic);
        case 2: return fillPointTofDiagnostic(&tofXshut5, "VL53L1X", diagnostic);
        case 3: return fillPointTofDiagnostic(&tofXshut6, "VL53L1X", diagnostic);
        case 4: return fillPointTofDiagnostic(&tofXshut7, "VL53L1X", diagnostic);
        case 5: return fillPointTofDiagnostic(&tofXshut8, "VL53L1X", diagnostic);
        default: return false;
    }
}

bool distance_sensors_xshut_expander_ready() {
    return xshutExpanderReady;
}

unsigned short distance_sensors_8x8_min_mm() {
    uint16_t grid[MATRIX_LIDAR_GRID_SIZE];
    if (!distance_sensors_get_8x8_grid(grid)) {
        return 0;
    }

    uint16_t closest = 0;
    for (uint8_t i = 0; i < MATRIX_LIDAR_GRID_SIZE; i++) {
        // 0 means "never populated", 4000 means "nothing in range" - neither
        // is a real measurement, so skip both.
        if (grid[i] == 0 || grid[i] >= MATRIX_LIDAR_NO_TARGET_MM) {
            continue;
        }
        if (closest == 0 || grid[i] < closest) {
            closest = grid[i];
        }
    }
    return closest;
}

bool distance_sensors_get_8x8_grid(unsigned short* buf) {
    if (buf == nullptr || !tof8x8.isGridAvailable()) {
        return false;
    }
    tof8x8.getGrid(buf);
    return true;
}

void distance_sensors_print_8x8_grid() {
    uint16_t grid[MATRIX_LIDAR_GRID_SIZE];
    tof8x8.getGrid(grid);

    Serial.println("tof_8x8 grid (mm):");
    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t col = 0; col < 8; col++) {
            Serial.print(grid[row * 8 + col]);
            Serial.print("\t");
        }
        Serial.println();
    }
}
