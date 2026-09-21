//************************************
//         sensor_config.h
//************************************
//
// Single source of truth for distance sensor wiring and I2C addressing.

#ifndef SENSOR_CONFIG_H_
#define SENSOR_CONFIG_H_

#include <Wire.h>

// 0x59/0x59 framed serial TOF sensor on Teensy Serial2 (RX2=D7, TX2=D8).
// The debug application uses the Teensy's USB Serial connection, leaving this
// hardware UART available for the sensor.
#define SERIAL_TOF_ENABLED 1
#define SERIAL_TOF_PORT Serial2
#define SERIAL_TOF_BAUD 115200

// The 7 VL53L0X/L1X sensors are on I2C bus 0 (Wire, pins 18/19).
#define VL53_I2C_BUS Wire
// The 8x8 array is on I2C bus 1 (Wire1, pins 16/17) - confirmed by scanning.
#define MATRIX_LIDAR_I2C_BUS Wire1

// All 7 VL53 XSHUT lines go through a SparkFun SX1509 I2C IO expander
// (not direct to Teensy GPIO) - these are SX1509 pin numbers (IO0-IO15),
// not Teensy pins. This order is also the sensor bring-up order (see
// SensorManager::beginAll).
#define SX1509_I2C_ADDRESS 0x3F
#define SX1509_I2C_BUS Wire

#define XSHUT0_PIN 0   // VL53L0X
#define XSHUT1_PIN 1   // VL53L0X
#define XSHUT2_PIN 2   // VL53L1X
#define XSHUT3_PIN 3   // VL53L1X
#define XSHUT4_PIN 4   // VL53L1X
#define XSHUT5_PIN 5   // VL53L1X
#define XSHUT6_PIN 6   // VL53L1X

// I2C addresses assigned once each sensor is brought out of reset (7-bit).
// 0x29 is every VL53L0X/L1X's shared power-on default and must never be
// reused once other sensors have been addressed - that's the whole reason
// for the one-at-a-time XSHUT bring-up sequence.
#define VL53L0X_ADDR_BASE 0x30   // -> 0x30, 0x31 for the 2 VL53L0X
#define VL53L1X_ADDR_BASE 0x35   // -> 0x35..0x39 for the 5 VL53L1X

#define MATRIX_LIDAR_ADDR 0x33   // fixed by the DFRobot module itself

// 4 analog Sharp-style IR distance sensors (see IRDistanceSensor.h for the
// distance conversion caveats).
#define IR0_PIN A6
#define IR1_PIN A7
#define IR2_PIN A8
#define IR3_PIN A9

// 2 HC-SR04-style ultrasonic sensors (Digital Raw 1, CON54). This exact
// pairing (30/31, 32/33) matches the previous year's working robot.
#define ULTRASONIC0_TRIG_PIN 30
#define ULTRASONIC0_ECHO_PIN 31
#define ULTRASONIC1_TRIG_PIN 32
#define ULTRASONIC1_ECHO_PIN 33

// PMW3901 optical flow sensor, default SPI bus (MOSI=11, MISO=12, SCK=13).
#define OPTICAL_FLOW_CS_PIN 10

// BNO055 IMU (SEN0253 combo board) on I2C bus 1 (Wire1).
#define IMU_I2C_ADDRESS 0x28
#define IMU_I2C_BUS Wire1

// LJ18A3-8-Z/BY inductive proximity sensor (metal detection).
#define INDUCTIVE_PIN A0

// Quadrature encoders, D2-D5. Interrupts on channel A only per encoder.
#define ENCODER0_PIN_A 2
#define ENCODER0_PIN_B 3
#define ENCODER1_PIN_A 4
#define ENCODER1_PIN_B 5

// Servo test connector (labelled "SERIAL7" - D28/D29 double as Serial7
// RX/TX, but here they're just being used as plain PWM outputs). Not sure
// yet which of the two is actually wired to the servo signal line, so
// both are driven identically until we confirm which one visibly moves it.
#define SERVO_D28_PIN 28
#define SERVO_D29_PIN 29

#endif /* SENSOR_CONFIG_H_ */
