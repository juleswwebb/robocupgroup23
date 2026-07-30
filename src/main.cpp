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

// Custom headers
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "return_to_base.h"
#include "DistanceSensors.h"
#include "OpticalFlow.h"

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
#define DISTANCE_SENSORS_PRINT_PERIOD       200
#define DISTANCE_SENSORS_PRINT_8X8_PERIOD  1000
#define OPTICAL_FLOW_UPDATE_PERIOD           20
#define OPTICAL_FLOW_PRINT_PERIOD           200

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

Scheduler taskManager;

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
  task_init();
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
