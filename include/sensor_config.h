//************************************
//         sensor_config.h
//************************************
//
// Single source of truth for distance sensor wiring and I2C addressing.

#ifndef SENSOR_CONFIG_H_
#define SENSOR_CONFIG_H_

#include <Wire.h>

// CH9143 matched Bluetooth serial bridge on Teensy Serial1.
// Serial1 pins: RX1=D0, TX1=D1. The bridge is transparent: the desktop app
// uses exactly the same newline-delimited JSON protocol over its virtual COM
// port as it does over the Teensy's direct USB Serial port.
#define BLUETOOTH_ENABLED 1
#define BLUETOOTH_PORT Serial1
#define BLUETOOTH_BAUD 115200

// Serial2 (RX2=D7, TX2=D8) is now the left/right drive ESC connector. A UART
// peripheral and Servo PWM cannot safely share those pins, so the old serial
// TOF is deliberately disabled until it is moved to another hardware serial
// port and these values are changed together.
#define SERIAL_TOF_ENABLED 0
#define SERIAL_TOF_PORT Serial2
#define SERIAL_TOF_BAUD 115200

// Main drive ESC / continuous-rotation-servo outputs on the connector labelled
// Serial2: RX2 D7 = left, TX2 D8 = right. Swap the two defines if the robot's
// physical left/right orientation proves opposite. 1050/1500/1950 us comes
// from the supplied working motor test sketch. The command watchdog makes both
// outputs neutral after 300 ms without a fresh command.
#define DRIVE_LEFT_PIN 7
#define DRIVE_RIGHT_PIN 8
// The installed ESC/motor direction is opposite the app's logical convention:
// a positive pulse previously drove the robot backwards. Keep the app/API
// convention intuitive (positive = forward) by reversing both outputs here.
#define DRIVE_LEFT_REVERSED 1
#define DRIVE_RIGHT_REVERSED 1
#define DRIVE_MIN_US 1050
#define DRIVE_NEUTRAL_US 1500
#define DRIVE_MAX_US 1950
#define DRIVE_US_PER_PERCENT 4.5f
#define DRIVE_COMMAND_TIMEOUT_MS 300
#define DRIVE_HARD_MAX_PERCENT 100
#define DRIVE_DEFAULT_MAX_PERCENT 35

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

// 2 HC-SR04-style ultrasonic sensors (Digital Raw 1, CON54). These remain
// disabled because their hardware/power path has not yet been validated.
#define ULTRASONIC_ENABLED 0
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

// Quadrature encoders on the input-capable Digital Raw 2 connector, matching
// the supplied encoder example and Group 7's verified wiring. Both channels
// are decoded: encoder 0 = D2/D3 and encoder 1 = D4/D5.
// Internal pull-ups support open-collector as well as 3.3 V push-pull outputs.
#define ENCODER0_PIN_A 2
#define ENCODER0_PIN_B 3
#define ENCODER1_PIN_A 4
#define ENCODER1_PIN_B 5
#define ENCODER_USE_INTERNAL_PULLUPS 1

// Servo test connector (labelled "SERIAL7" - D28/D29 double as Serial7
// RX/TX, but here they're just being used as plain PWM outputs). Not sure
// yet which of the two is actually wired to the servo signal line, so
// both are driven identically until we confirm which one visibly moves it.
#define SERVO_D28_PIN 28
#define SERVO_D29_PIN 29

#endif /* SENSOR_CONFIG_H_ */
