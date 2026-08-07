/********************************************************************************
 *                               ROBOCUP TEMPLATE
 *
 *  This is a template program design with modules for
 *  different components of the robot, and a task scheduler
 *  for controlling how frequently tasks should run
 *
 *  adapted from the ENMT301 robocup_template
 ******************************************************************************/

#include <Arduino.h>
#include <Servo.h>                  //control the DC motors
//#include <Herkulex.h>             //smart servo
#include <Adafruit_TCS34725.h>      //colour sensor
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler
#include <string.h>
#include <stdlib.h>

// Custom headers
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "return_to_base.h"
#include "DistanceSensors.h"
#include "OpticalFlow.h"
#include "IMU.h"
#include "Inductive.h"
#include "Encoders.h"
#include "ServoControl.h"
#include "Console.h"

//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Task period Definitions
// ALL OF THESE VALUES WILL NEED TO BE SET TO SOMETHING USEFUL !!!!!!!!!!!!!!!!!!!!
#define US_READ_TASK_PERIOD                 40
#define IR_READ_TASK_PERIOD                 40
#define COLOUR_READ_TASK_PERIOD             60 // must be >= the sensor's integration time (50ms), see ColourSensor.h
#define SENSOR_AVERAGE_PERIOD               40
#define SET_MOTOR_TASK_PERIOD               40
#define WEIGHT_SCAN_TASK_PERIOD             40
#define COLLECT_WEIGHT_TASK_PERIOD          40
#define RETURN_TO_BASE_TASK_PERIOD          40
#define DETECT_BASE_TASK_PERIOD             40
#define UNLOAD_WEIGHTS_TASK_PERIOD          40
#define DISTANCE_SENSORS_UPDATE_PERIOD       20
#define OPTICAL_FLOW_UPDATE_PERIOD           20
#define IMU_UPDATE_PERIOD                    20
#define INDUCTIVE_UPDATE_PERIOD              20
#define CONSOLE_UPDATE_PERIOD                20 // how often we check for typed commands

// Single knob for how often sensor readings get printed to Serial - turn
// this up if the monitor is scrolling faster than you can read. This is
// separate from the *_UPDATE_PERIOD values above, so sensors keep
// sampling at full speed internally regardless of print rate.
#define SERIAL_PRINT_PERIOD                1000
#define COLOUR_PRINT_PERIOD                 SERIAL_PRINT_PERIOD
#define DISTANCE_SENSORS_PRINT_PERIOD       SERIAL_PRINT_PERIOD
#define DISTANCE_SENSORS_PRINT_8X8_PERIOD  (SERIAL_PRINT_PERIOD * 3)
#define OPTICAL_FLOW_PRINT_PERIOD           SERIAL_PRINT_PERIOD
#define IMU_PRINT_PERIOD                    SERIAL_PRINT_PERIOD
#define INDUCTIVE_PRINT_PERIOD              SERIAL_PRINT_PERIOD
#define ENCODERS_PRINT_PERIOD               SERIAL_PRINT_PERIOD
#define SERVO_CONTROL_PRINT_PERIOD          SERIAL_PRINT_PERIOD

// Task execution amount definitions
// -1 means indefinitely
#define US_READ_TASK_NUM_EXECUTE           -1
#define IR_READ_TASK_NUM_EXECUTE           -1
#define COLOUR_READ_TASK_NUM_EXECUTE       -1
#define SENSOR_AVERAGE_NUM_EXECUTE         -1
#define SET_MOTOR_TASK_NUM_EXECUTE         -1
#define WEIGHT_SCAN_TASK_NUM_EXECUTE       -1
#define COLLECT_WEIGHT_TASK_NUM_EXECUTE    -1
#define RETURN_TO_BASE_TASK_NUM_EXECUTE    -1
#define DETECT_BASE_TASK_NUM_EXECUTE       -1
#define UNLOAD_WEIGHTS_TASK_NUM_EXECUTE    -1
#define DISTANCE_SENSORS_NUM_EXECUTE       -1
#define OPTICAL_FLOW_NUM_EXECUTE           -1
#define IMU_NUM_EXECUTE                    -1
#define INDUCTIVE_NUM_EXECUTE              -1
#define SERVO_CONTROL_NUM_EXECUTE          -1
#define CONSOLE_NUM_EXECUTE                -1
#define ENCODERS_NUM_EXECUTE               -1

// Pin definitions
#define IO_POWER  49

// Serial definitions
#define BAUD_RATE 115200

Servo right_motor;
Servo left_motor;

//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

/* The first value is the period, second is how many times it executes
   (-1 means indefinitely), third one is the callback function */

// Tasks for reading sensors
Task tRead_ultrasonic(US_READ_TASK_PERIOD,       US_READ_TASK_NUM_EXECUTE,        &read_ultrasonic);
Task tRead_infrared(IR_READ_TASK_PERIOD,         IR_READ_TASK_NUM_EXECUTE,        &read_infrared);
Task tRead_colour(COLOUR_READ_TASK_PERIOD,       COLOUR_READ_TASK_NUM_EXECUTE,    &read_colour);
Task tPrint_colour(COLOUR_PRINT_PERIOD,          COLOUR_READ_TASK_NUM_EXECUTE,    &colour_print);
Task tSensor_average(SENSOR_AVERAGE_PERIOD,      SENSOR_AVERAGE_NUM_EXECUTE,      &sensor_average);

// Task to set the motor speeds and direction
Task tSet_motor(SET_MOTOR_TASK_PERIOD,           SET_MOTOR_TASK_NUM_EXECUTE,      &set_motor);

// Tasks to scan for weights and collection upon detection
Task tWeight_scan(WEIGHT_SCAN_TASK_PERIOD,       WEIGHT_SCAN_TASK_NUM_EXECUTE,    &weight_scan);
Task tCollect_weight(COLLECT_WEIGHT_TASK_PERIOD, COLLECT_WEIGHT_TASK_NUM_EXECUTE, &collect_weight);

// Tasks to search for bases and unload weights
Task tReturn_to_base(RETURN_TO_BASE_TASK_PERIOD, RETURN_TO_BASE_TASK_NUM_EXECUTE, &return_to_base);
Task tDetect_base(DETECT_BASE_TASK_PERIOD,       DETECT_BASE_TASK_NUM_EXECUTE,    &detect_base);
Task tUnload_weights(UNLOAD_WEIGHTS_TASK_PERIOD, UNLOAD_WEIGHTS_TASK_NUM_EXECUTE, &unload_weights);

// Tasks for the TOF sensor subsystem (see DistanceSensors.h/.cpp)
Task tUpdate_distance_sensors(DISTANCE_SENSORS_UPDATE_PERIOD, DISTANCE_SENSORS_NUM_EXECUTE, &distance_sensors_update);
Task tPrint_distance_sensors(DISTANCE_SENSORS_PRINT_PERIOD,   DISTANCE_SENSORS_NUM_EXECUTE, &distance_sensors_print);
Task tPrint_8x8_grid(DISTANCE_SENSORS_PRINT_8X8_PERIOD,       DISTANCE_SENSORS_NUM_EXECUTE, &distance_sensors_print_8x8_grid);

// Tasks for the optical flow sensor (see OpticalFlow.h/.cpp)
Task tUpdate_optical_flow(OPTICAL_FLOW_UPDATE_PERIOD, OPTICAL_FLOW_NUM_EXECUTE, &optical_flow_update);
Task tPrint_optical_flow(OPTICAL_FLOW_PRINT_PERIOD,   OPTICAL_FLOW_NUM_EXECUTE, &optical_flow_print);

// Tasks for the IMU (see IMU.h/.cpp)
Task tUpdate_imu(IMU_UPDATE_PERIOD, IMU_NUM_EXECUTE, &imu_update);
Task tPrint_imu(IMU_PRINT_PERIOD,   IMU_NUM_EXECUTE, &imu_print);

// Tasks for the inductive proximity sensor (see Inductive.h/.cpp)
Task tUpdate_inductive(INDUCTIVE_UPDATE_PERIOD, INDUCTIVE_NUM_EXECUTE, &inductive_update);
Task tPrint_inductive(INDUCTIVE_PRINT_PERIOD,   INDUCTIVE_NUM_EXECUTE, &inductive_print);

// Task for the encoders (see Encoders.h/.cpp) - no update task, position is
// kept current by interrupts in the background.
Task tPrint_encoders(ENCODERS_PRINT_PERIOD, ENCODERS_NUM_EXECUTE, &encoders_print);

// Task to print the servo's current commanded state (see ServoControl.h/.cpp)
// - no update task, it's set directly by console commands, nothing to poll.
Task tPrint_servo_control(SERVO_CONTROL_PRINT_PERIOD, SERVO_CONTROL_NUM_EXECUTE, &servo_control_print);

// Task for the serial command console (see Console.h/.cpp)
Task tUpdate_console(CONSOLE_UPDATE_PERIOD, CONSOLE_NUM_EXECUTE, &console_update);

Scheduler taskManager;

//**********************************************************************************
// Serial command console - lets you switch between normal sensor-debug
// printing and a quieter "test" mode for driving the servo by hand. The
// sensors keep updating internally in both modes; only the print tasks
// (i.e. how much scrolls past in the serial monitor) get toggled.
//
// Commands:
//   mode sensors          - show all sensor debug prints (default)
//   mode test             - hide sensor prints, just show servo state
//   servo us <500-2500>   - set the servo's raw pulse width directly
//   servo speed <-100..100> - set speed as a percentage (0 = stop)
//   servo angle <0-180>   - set a position (for a positional servo)
//   servo stop            - shorthand for "servo speed 0"
//   help                  - show this list
//**********************************************************************************
static bool testMode = false;

static void set_sensor_debug_prints_enabled(bool enabled) {
  if (enabled) {
    tPrint_colour.enable();
    tPrint_distance_sensors.enable();
    tPrint_8x8_grid.enable();
    tPrint_optical_flow.enable();
    tPrint_imu.enable();
    tPrint_inductive.enable();
    tPrint_encoders.enable();
  } else {
    tPrint_colour.disable();
    tPrint_distance_sensors.disable();
    tPrint_8x8_grid.disable();
    tPrint_optical_flow.disable();
    tPrint_imu.disable();
    tPrint_inductive.disable();
    tPrint_encoders.disable();
  }
}

static void print_console_help() {
  Serial.println("Commands:");
  Serial.println("  mode sensors            - show all sensor debug prints (default)");
  Serial.println("  mode test               - hide sensor prints, just show servo state");
  Serial.println("  servo us <500-2500>     - set the servo's raw pulse width directly");
  Serial.println("  servo speed <-100..100> - set speed as a percentage (0 = stop)");
  Serial.println("  servo angle <0-180>     - set a position (for a positional servo)");
  Serial.println("  servo stop              - shorthand for \"servo speed 0\"");
  Serial.println("  help                    - show this list");
}

static void handle_console_command(const char* command, const char* args) {
  if (strcmp(command, "mode") == 0) {
    if (strcmp(args, "test") == 0) {
      testMode = true;
      set_sensor_debug_prints_enabled(false);
      Serial.println("mode: test (sensor prints hidden, servo prints still shown)");
    } else if (strcmp(args, "sensors") == 0) {
      testMode = false;
      set_sensor_debug_prints_enabled(true);
      Serial.println("mode: sensors (all debug prints shown)");
    } else {
      Serial.println("usage: mode <sensors|test>");
    }
  } else if (strcmp(command, "servo") == 0) {
    // Manual split on the first space instead of sscanf("%s %d", ...) -
    // sscanf pulls in newlib's whole format parser for a trivial job.
    char sub[16] = {0};
    const char* valueText = strchr(args, ' ');
    size_t subLen = valueText ? (size_t)(valueText - args) : strlen(args);
    if (subLen >= sizeof(sub)) subLen = sizeof(sub) - 1;
    memcpy(sub, args, subLen);
    sub[subLen] = '\0';
    int value = valueText ? atoi(valueText + 1) : 0;
    bool hasValue = (valueText != nullptr);

    if (strcmp(sub, "us") == 0 && hasValue) {
      servo_control_set_microseconds(value);
    } else if (strcmp(sub, "speed") == 0 && hasValue) {
      servo_control_set_speed(value);
    } else if (strcmp(sub, "angle") == 0 && hasValue) {
      servo_control_set_angle(value);
    } else if (strcmp(sub, "stop") == 0) {
      servo_control_set_speed(0);
    } else {
      Serial.println("usage: servo <us|speed|angle> <value>   or   servo stop");
    }
    servo_control_print();
  } else if (strcmp(command, "help") == 0) {
    print_console_help();
  } else {
    Serial.print("unknown command: ");
    Serial.println(command);
    print_console_help();
  }
}

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void robot_init();
void task_init();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  robot_init();
  distance_sensors_init(); // brings up Wire + all TOF sensors
  sensors_colour_init();   // brings up Wire1 colour sensor
  optical_flow_init();     // brings up SPI + the optical flow sensor
  imu_init();              // brings up the Wire1 IMU (BNO055)
  inductive_init();        // brings up the inductive proximity sensor pin
  encoders_init();         // brings up the encoder pins + interrupts
  servo_control_init();    // attaches the D28/D29 servo test pins
  console_set_command_handler(&handle_console_command);
  console_init();
  task_init();
  print_console_help();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work)
// Set as high or low
//**********************************************************************************
void pin_init() {
  Serial.println("Pins have been initialised \n");

  pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
  digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
}

//**********************************************************************************
// Set default robot state
//**********************************************************************************
void robot_init() {
  Serial.println("Robot is ready \n");
}

//**********************************************************************************
// Initialise the tasks for the scheduler
//**********************************************************************************
void task_init() {

  // Initialise the task scheduler
  taskManager.init();

  // Add tasks to the scheduler
  //
  // The stub modules (ultrasonic/infrared/colour/motors/weights/base) don't
  // have real logic yet - just a Serial.println placeholder each - so
  // they're left out of the scheduler for now to keep the serial monitor
  // readable while we bring up the TOF sensors. Add them back in as each
  // module gets implemented for real.
  // taskManager.addTask(tRead_ultrasonic);
  // taskManager.addTask(tRead_infrared);
  taskManager.addTask(tRead_colour);
  taskManager.addTask(tPrint_colour);
  // taskManager.addTask(tSensor_average);
  // taskManager.addTask(tSet_motor);
  // taskManager.addTask(tWeight_scan);
  // taskManager.addTask(tCollect_weight);
  // taskManager.addTask(tReturn_to_base);
  // taskManager.addTask(tDetect_base);
  // taskManager.addTask(tUnload_weights);
  taskManager.addTask(tUpdate_distance_sensors);
  taskManager.addTask(tPrint_distance_sensors);
  taskManager.addTask(tPrint_8x8_grid);
  taskManager.addTask(tUpdate_optical_flow);
  taskManager.addTask(tPrint_optical_flow);
  taskManager.addTask(tUpdate_imu);
  taskManager.addTask(tPrint_imu);
  taskManager.addTask(tUpdate_inductive);
  taskManager.addTask(tPrint_inductive);
  taskManager.addTask(tPrint_encoders);
  taskManager.addTask(tPrint_servo_control);
  taskManager.addTask(tUpdate_console);

  // Enable the tasks
  taskManager.enableAll();

  Serial.println("Tasks have been initialised \n");
}

//**********************************************************************************
// put your main code here, to run repeatedly:
//**********************************************************************************
void loop() {
  taskManager.execute();    //execute the scheduler
}
