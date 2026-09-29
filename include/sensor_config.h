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
#define DRIVE_DEFAULT_MAX_PERCENT 100

// Drum motor-driver signal outputs on the user's D28/D29 connector.
// These replace the old servo test outputs; do not attach ServoControl here.
// Confirm left/right direction and driver pulse requirements before running.
#define DRUM_LEFT_PIN 28
#define DRUM_RIGHT_PIN 29
#define DRUM_COMMAND_TIMEOUT_MS 300
#define DRUM_TEST_MAX_PERCENT 100

// Electromagnet driver input. The coil must use its own supply through a
// MOSFET/relay driver; this pin only drives the driver's logic input.
#define MAGNET_PIN 26
#define MAGNET_COMMAND_TIMEOUT_MS 1000

// Servo test output on D20. D20 is also A6, so the unused IR reader must not
// sample it while this output is attached.
#define SERVO_TEST_PIN 20
#define SERVO_TEST_NEUTRAL_US 1500
#define SERVO_TEST_MIN_US 1000
#define SERVO_TEST_MAX_US 2000
#define SERVO_TEST_COMMAND_TIMEOUT_MS 300
#define IR_DISTANCE_SENSORS_ENABLED 0

// The six point VL53L1X sensors and their SX1509 are on I2C bus 0
// (Wire, pins 18/19).
#define VL53_I2C_BUS Wire
// The 8x8 array is on I2C bus 1 (Wire1, pins 16/17) - confirmed by scanning.
#define MATRIX_LIDAR_I2C_BUS Wire1

// Six point VL53L1X ToFs use the add-on SX1509 at 0x71 on IO5-IO10.
// These are expander pins, not Teensy GPIO pins.
#define SX1509_I2C_ADDRESS 0x71
#define SX1509_I2C_BUS Wire

#define TOF_XSHUT3_IO 8
#define TOF_XSHUT4_IO 5
#define TOF_XSHUT5_IO 6
#define TOF_XSHUT6_IO 7
#define TOF_XSHUT7_IO 9
#define TOF_XSHUT8_IO 10

// I2C addresses assigned once each sensor is brought out of reset (7-bit).
// 0x29 is every VL53L0X/L1X's shared power-on default and must never be
// reused once other sensors have been addressed - that's the whole reason
// for the one-at-a-time XSHUT bring-up sequence.
#define VL53L1X_ADDR_BASE 0x35   // -> 0x35..0x3A for six VL53L1X

#define MATRIX_LIDAR_ADDR 0x33   // fixed by the DFRobot module itself

// Legacy IR input pin assignments. IR_DISTANCE_SENSORS_ENABLED is currently
// zero because D20/A6 is assigned to the servo output.
#define IR0_PIN A6
#define IR1_PIN A7
#define IR2_PIN A8
#define IR3_PIN A9

// 2 HC-SR04-style ultrasonic sensors (Digital Raw 1, CON54).
// Echo must be level-shifted to 3.3 V before reaching the Teensy 4.0.
#define ULTRASONIC_ENABLED 1
#define ULTRASONIC0_TRIG_PIN 30
#define ULTRASONIC0_ECHO_PIN 31
#define ULTRASONIC1_TRIG_PIN 32
#define ULTRASONIC1_ECHO_PIN 33

// PMW3901 optical flow sensor, default SPI bus (MOSI=11, MISO=12, SCK=13).
#define OPTICAL_FLOW_CS_PIN 10

// BNO055 IMU (SEN0253 combo board) on I2C bus 0 (Wire, pins 18/19).
#define IMU_I2C_ADDRESS 0x28
#define IMU_I2C_BUS Wire

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
// Measured straight-line calibration used by the onboard Mission Planner
// follower. GUI calibration values are uploaded with each mission and can
// override these defaults for that run.
#define ENCODER0_MM_PER_COUNT (3635.0f / 41153.0f)
#define ENCODER1_MM_PER_COUNT (3635.0f / 42224.0f)
#define ENCODER0_REVERSED 0
#define ENCODER1_REVERSED 1

// D20 is the servo test output; D21 is unused by this servo. D28/D29 remain
// dedicated to the left/right drum motor drivers.

#endif /* SENSOR_CONFIG_H_ */
