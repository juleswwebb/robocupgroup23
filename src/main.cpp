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
#include "Navigation.h"
#include "ServoControl.h"
#include "DriveControl.h"
#include "DrumControl.h"
#include "MagnetControl.h"
#include "Console.h"
#include "DebugProtocol.h"
#include "sensor_config.h"

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
// Polled fast; the protocol self-paces to its own telemetry.interval_ms
// parameter, which the GUI can retune live.
#define DEBUG_PROTOCOL_UPDATE_PERIOD         10
#define DRIVE_CONTROL_UPDATE_PERIOD           10

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
#define DEBUG_PROTOCOL_NUM_EXECUTE         -1
#define DRIVE_CONTROL_NUM_EXECUTE          -1
#define ENCODERS_NUM_EXECUTE               -1
#define NAVIGATION_UPDATE_PERIOD            50
#define NAVIGATION_NUM_EXECUTE              -1

// Pin definitions
#define IO_POWER  49

// Serial definitions
#define BAUD_RATE 115200

Servo right_motor;
Servo left_motor;

#if BLUETOOTH_ENABLED
// Persistent extra Serial1 RX storage. The CH9143 can deliver commands while
// the Teensy is transmitting a large telemetry frame; the default UART buffer
// is too small for that burst.
static uint8_t bluetoothRxBuffer[2048];
static uint8_t bluetoothTxBuffer[8192];

#endif

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

// Task for the serial command console (see Console.h/.cpp)
Task tUpdate_console(CONSOLE_UPDATE_PERIOD, CONSOLE_NUM_EXECUTE, &console_update);

// Task for the JSON debug protocol used by tools/debug_gui (see DebugProtocol.h/.cpp)
Task tUpdate_debug_protocol(DEBUG_PROTOCOL_UPDATE_PERIOD, DEBUG_PROTOCOL_NUM_EXECUTE, &debug_protocol_update);
// Enforces the short command watchdog that makes the drive outputs neutral
// if the GUI connection or a keyboard event disappears.
Task tUpdate_drive_control(DRIVE_CONTROL_UPDATE_PERIOD, DRIVE_CONTROL_NUM_EXECUTE, &drive_control_update);
Task tUpdate_drum_control(DRIVE_CONTROL_UPDATE_PERIOD, DRIVE_CONTROL_NUM_EXECUTE, &drum_control_update);
Task tUpdate_servo_control(DRIVE_CONTROL_UPDATE_PERIOD, DRIVE_CONTROL_NUM_EXECUTE, &servo_control_update);
Task tUpdate_magnet_control(DRIVE_CONTROL_UPDATE_PERIOD, DRIVE_CONTROL_NUM_EXECUTE, &magnet_control_update);
Task tUpdate_navigation(NAVIGATION_UPDATE_PERIOD, NAVIGATION_NUM_EXECUTE, &navigation_update);

Scheduler taskManager;

//**********************************************************************************
// Serial command console - lets you switch between normal sensor-debug
// printing and a quieter "test" mode for driving the servo by hand. The
// sensors keep updating internally in both modes; only the print tasks
// (i.e. how much scrolls past in the serial monitor) get toggled.
//
// Commands:
//   mode sensors          - show all sensor debug prints (default)
//   mode test             - hide sensor prints for actuator tests
//   servo us <1000-2000>  - pulse/speed test on D20 (1500 = neutral)
//   servo angle <0-180>   - positional-servo target on D20
//   servo stop            - neutralise pulse-test mode; position mode holds
//   drive <left> <right>  - command both main drive motors (-100..100)
//   drive stop            - neutral both main drive motors
//   help                  - show this list
//
// The Python debug console (tools/debug_gui) talks JSON over this same
// port instead; connecting it switches the firmware into JSON mode
// automatically and silences these text prints. See DebugProtocol.h.
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

// Called by DebugProtocol when the GUI connects/disconnects. JSON telemetry
// and human-readable prints can't share the port, so they're mutually
// exclusive - the sensors themselves keep updating either way.
static void on_debug_json_mode_changed(bool json_active) {
  if (json_active) {
    set_sensor_debug_prints_enabled(false);
  } else {
    drum_control_stop();
    servo_control_stop();
    set_sensor_debug_prints_enabled(!testMode);
  }
}

static void print_console_help() {
  Print& out = console_output();
  out.println("Commands:");
  out.println("  mode sensors            - show all sensor debug prints (default)");
  out.println("  mode test               - hide sensor prints for actuator testing");
  out.println("  drive <left> <right>    - main drive motors, -100 to 100");
  out.println("  drive stop              - neutral both main drive motors");
  out.println("  servo us <1000-2000>    - pulse/speed test on D20 (1500 = neutral)");
  out.println("  servo angle <0-180>     - positional-servo target on D20");
  out.println("  servo stop              - neutralise pulse-test mode");
  out.println("  help                    - show this list");
}

static void handle_console_command(const char* command, const char* args) {
  Print& out = console_output();
  if (strcmp(command, "mode") == 0) {
    if (strcmp(args, "test") == 0) {
      testMode = true;
      set_sensor_debug_prints_enabled(false);
      out.println("mode: test (sensor prints hidden)");
    } else if (strcmp(args, "sensors") == 0) {
      testMode = false;
      // Also drops JSON mode, so this is the way back to readable output
      // if the debug GUI disconnected without saying goodbye.
      debug_protocol_set_active(false);
      set_sensor_debug_prints_enabled(true);
      out.println("mode: sensors (all debug prints shown)");
    } else {
      out.println("usage: mode <sensors|test>");
    }
  } else if (strcmp(command, "servo") == 0) {
    if (strcmp(args, "stop") == 0) {
      servo_control_stop();
    } else {
      const char* separator = strchr(args, ' ');
      if (separator == nullptr) {
        out.println("usage: servo us <1000-2000> | servo angle <0-180> | servo stop");
        return;
      }
      if (strncmp(args, "angle ", 6) == 0) {
        servo_control_set_angle(atoi(separator + 1));
      } else if (strncmp(args, "us ", 3) == 0) {
        const int pulseUs = atoi(separator + 1);
        if (pulseUs < SERVO_TEST_MIN_US || pulseUs > SERVO_TEST_MAX_US) {
          out.println("servo: pulse must be 1000-2000 us");
          return;
        }
        servo_control_set_microseconds(pulseUs);
      } else {
        out.println("usage: servo us <1000-2000> | servo angle <0-180> | servo stop");
        return;
      }
    }
    servo_control_print();
  } else if (strcmp(command, "drive") == 0) {
    if (strcmp(args, "stop") == 0) {
      drive_control_stop();
    } else {
      const char* separator = strchr(args, ' ');
      if (separator == nullptr) {
        out.println("usage: drive <left> <right>   or   drive stop");
        return;
      }
      drive_control_set_percent(atoi(args), atoi(separator + 1));
    }
    out.print("drive: left=");
    out.print(drive_control_get_left_percent());
    out.print(" right=");
    out.println(drive_control_get_right_percent());
  } else if (strcmp(command, "help") == 0) {
    print_console_help();
  } else {
    out.print("unknown command: ");
    out.println(command);
    print_console_help();
  }
}

//**********************************************************************************
// Function Definitions
//*********************************************************************************
void pin_init();
void robot_init();
void task_init();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
#if BLUETOOTH_ENABLED
  BLUETOOTH_PORT.addMemoryForRead(bluetoothRxBuffer, sizeof(bluetoothRxBuffer));
  // Match the proven Group 7 transport: queue complete JSON lines in a large
  // hardware UART ring and let Serial1 drain them asynchronously.
  BLUETOOTH_PORT.addMemoryForWrite(bluetoothTxBuffer, sizeof(bluetoothTxBuffer));
  BLUETOOTH_PORT.begin(BLUETOOTH_BAUD);
#endif
  pin_init();
  robot_init();
  distance_sensors_init(); // brings up Wire + all TOF sensors
  sensors_colour_init();   // brings up Wire1 colour sensor
  optical_flow_init();     // brings up SPI + the optical flow sensor
  imu_init();              // brings up the Wire1 IMU (BNO055)
  inductive_init();        // brings up the inductive proximity sensor pin
  encoders_init();         // brings up the encoder pins + interrupts
  drive_control_init();    // D7/D8 drive ESCs; starts safely at neutral
  drum_control_init();     // D28/D29 drum outputs, neutral at boot
  magnet_control_init();   // D26 magnet driver output, safely OFF at boot
  servo_control_init();    // D20 servo/pulse output, neutral at boot
  navigation_init();       // autonomous navigation remains disabled at boot
  console_set_command_handler(&handle_console_command);
  console_set_json_handler(&debug_protocol_handle_json);
#if BLUETOOTH_ENABLED
  console_set_bluetooth_json_handler(&debug_protocol_handle_bluetooth_json);
#endif
  console_init();

  debug_protocol_set_mode_changed_handler(&on_debug_json_mode_changed);
  debug_protocol_init();

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
  // Safety and communications run first on every scheduler pass. If a sensor
  // driver is slow or faulty, STOP/drive watchdog and app commands must still
  // be serviced before entering that driver.
  taskManager.addTask(tUpdate_drive_control);
  taskManager.addTask(tUpdate_drum_control);
  taskManager.addTask(tUpdate_servo_control);
  taskManager.addTask(tUpdate_magnet_control);
  taskManager.addTask(tUpdate_console);
  taskManager.addTask(tUpdate_debug_protocol);
  taskManager.addTask(tUpdate_navigation);
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
