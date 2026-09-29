#include "DebugProtocol.h"
#include "DistanceSensors.h"
#include "OpticalFlow.h"
#include "IMU.h"
#include "Inductive.h"
#include "Encoders.h"
#include "ServoControl.h"
#include "DriveControl.h"
#include "DrumControl.h"
#include "MagnetControl.h"
#include "Navigation.h"
#include "ExplorationMode.h"
#include "MissionNavigation.h"
#include "Console.h"
#include "sensor_config.h"
#include "sensors.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>

// USB Serial and the CH9143 on Serial1 carry the same protocol. One transport
// is selected by the most recent hello/request from an app; all replies and
// telemetry go back through that same transport.
enum class DebugTransport : uint8_t { Usb, Bluetooth };
static DebugTransport activeTransport = DebugTransport::Usb;
static Print* protocolOutput = &Serial;

#define TELEMETRY_INTERVAL_MIN_MS 20
#define TELEMETRY_INTERVAL_MAX_MS 2000
#define BLUETOOTH_TELEMETRY_MIN_MS 500
#define BLUETOOTH_TX_BUFFER_SIZE 4096

static bool jsonActive = false;
static bool debugMode = false;
static unsigned long telemetryIntervalMs = 100; // 10 Hz; USB CDC has ample room for the 8x8 frame
static unsigned long lastTelemetryMs = 0;
static unsigned long lastDefinitionsMs = 0;
static unsigned long bluetoothRxMessages = 0;
static DebugModeChangedHandler modeChangedHandler = nullptr;
static char bluetoothTxBuffer[BLUETOOTH_TX_BUFFER_SIZE];

static void select_transport(DebugTransport transport) {
    activeTransport = transport;
    protocolOutput = (transport == DebugTransport::Bluetooth)
        ? static_cast<Print*>(&BLUETOOTH_PORT)
        : static_cast<Print*>(&Serial);
    if (transport == DebugTransport::Bluetooth &&
        telemetryIntervalMs < BLUETOOTH_TELEMETRY_MIN_MS) {
        telemetryIntervalMs = BLUETOOTH_TELEMETRY_MIN_MS;
    }
}

static unsigned long active_telemetry_minimum_ms() {
    return activeTransport == DebugTransport::Bluetooth
        ? BLUETOOTH_TELEMETRY_MIN_MS
        : TELEMETRY_INTERVAL_MIN_MS;
}

void debug_protocol_set_mode_changed_handler(DebugModeChangedHandler handler) {
    modeChangedHandler = handler;
}

void debug_protocol_init() {
    jsonActive = false;
    debugMode = false;
    lastTelemetryMs = millis();
}

void debug_protocol_update() {
    debug_protocol_send_telemetry();
}

bool debug_protocol_is_active() {
    return jsonActive;
}

// =====================================================================
// Sending
// =====================================================================

static void send(JsonDocument& doc) {
    if (activeTransport == DebugTransport::Bluetooth) {
        const size_t required = measureJson(doc);
        if (required >= sizeof(bluetoothTxBuffer)) {
            return;
        }
        const size_t length = serializeJson(doc, bluetoothTxBuffer,
                                            sizeof(bluetoothTxBuffer));
        BLUETOOTH_PORT.write(
            reinterpret_cast<const uint8_t*>(bluetoothTxBuffer), length);
        BLUETOOTH_PORT.write('\n');
        return;
    }
    serializeJson(doc, *protocolOutput);
    protocolOutput->println();
}

void debug_protocol_log(const char* level, const char* message) {
    if (!jsonActive) {
        Serial.print("[");
        Serial.print(level);
        Serial.print("] ");
        Serial.println(message);
        return;
    }

    JsonDocument doc;
    doc["type"] = "log";
    doc["level"] = level;
    doc["message"] = message;
    send(doc);
}

static void send_error(const char* message) {
    if (!jsonActive) {
        return;
    }
    JsonDocument doc;
    doc["type"] = "error";
    doc["message"] = message;
    send(doc);
}

static void send_state() {
    if (!jsonActive) {
        return;
    }
    JsonDocument doc;
    doc["type"] = "state";
    doc["debug_mode"] = debugMode;
    doc["stopped"] = (!drive_control_is_active() && !drum_control_is_active()
                      && !magnet_control_is_on() && !servo_control_is_active());
    doc["magnet_on"] = magnet_control_is_on();
    doc["servo_pin"] = servo_control_get_pin();
    doc["servo_pulse_us"] = servo_control_get_microseconds();
    doc["servo_angle_deg"] = servo_control_get_angle();
    doc["servo_position_mode"] = servo_control_is_position_mode();
    doc["fault"] = false;
    doc["uptime_ms"] = millis();
    send(doc);
}

static void send_parameter_value(const char* name, long value) {
    if (!jsonActive) {
        return;
    }
    JsonDocument doc;
    doc["type"] = "parameter_value";
    doc["name"] = name;
    doc["value"] = value;
    send(doc);
}

static void dotted_name(const char* name, char* out, size_t outSize);

// ---------------------------------------------------------------------
// Definitions - these are what let the GUI build its own controls, so
// adding a tunable or a test routine never needs a Python change.
// ---------------------------------------------------------------------

static void send_telemetry_definition(const char* name,
                                      const char* label,
                                      const char* group,
                                      const char* unit,
                                      bool plottable = true) {
    JsonDocument doc;
    doc["type"] = "telemetry_definition";
    doc["name"] = name;
    doc["label"] = label;
    doc["group"] = group;
    doc["unit"] = unit;
    doc["plottable"] = plottable;
    send(doc);
}

static const char* distance_group(const char* dottedName) {
    if (strncmp(dottedName, "ir.", 3) == 0) return "Infrared distance";
    if (strncmp(dottedName, "ultrasonic.", 11) == 0) return "Ultrasonic";
    return "Time of flight";
}

static void send_telemetry_definitions() {
    char name[40];
    for (unsigned char i = 0; i < distance_sensors_count(); i++) {
        DistanceSensor* sensor = distance_sensor_get_by_index(i);
        if (sensor == nullptr) continue;
        dotted_name(sensor->getName(), name, sizeof(name));
        send_telemetry_definition(name, sensor->getName(), distance_group(name), "mm");
    }

    send_telemetry_definition("tof.xshut_expander_ok", "XSHUT expander found",
                              "ToF diagnostics", "bool", false);
    for (uint8_t i = 0; i < distance_sensors_point_tof_count(); i++) {
        PointTofDiagnostic diagnostic;
        if (!distance_sensors_get_point_tof_diagnostic(i, &diagnostic)) continue;
        char prefix[40];
        char signal[64];
        dotted_name(diagnostic.name, prefix, sizeof(prefix));
        snprintf(signal, sizeof(signal), "%s.init_ok", prefix);
        send_telemetry_definition(signal, "VL53 initialized", "ToF diagnostics", "bool", false);
        snprintf(signal, sizeof(signal), "%s.range_status", prefix);
        send_telemetry_definition(signal, "VL53 range status", "ToF diagnostics", "code", false);
        snprintf(signal, sizeof(signal), "%s.i2c_status", prefix);
        send_telemetry_definition(signal, "VL53 I2C status", "ToF diagnostics", "code", false);
        snprintf(signal, sizeof(signal), "%s.model_id", prefix);
        send_telemetry_definition(signal, "VL53 model ID", "ToF diagnostics", "hex", false);
    }

    send_telemetry_definition("tof.array_min", "8x8 nearest valid zone", "8x8 TOF", "mm");
    send_telemetry_definition("tof.array_valid_zones", "8x8 valid zones", "8x8 TOF", "zones", false);
    send_telemetry_definition("tof.array_frame_ok", "8x8 frame read OK", "8x8 TOF", "", false);

    char zoneName[24];
    char zoneLabel[24];
    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t col = 0; col < 8; col++) {
            snprintf(zoneName, sizeof(zoneName), "tof.array.r%uc%u", row, col);
            snprintf(zoneLabel, sizeof(zoneLabel), "Zone R%u C%u", row, col);
            send_telemetry_definition(zoneName, zoneLabel, "8x8 TOF zones", "mm");
        }
    }

    send_telemetry_definition("colour.r", "Red", "Colour", "raw");
    send_telemetry_definition("colour.g", "Green", "Colour", "raw");
    send_telemetry_definition("colour.b", "Blue", "Colour", "raw");
    send_telemetry_definition("colour.c", "Clear", "Colour", "raw");

    send_telemetry_definition("imu.heading", "Heading", "IMU", "deg");
    send_telemetry_definition("imu.roll", "Roll", "IMU", "deg");
    send_telemetry_definition("imu.pitch", "Pitch", "IMU", "deg");
    send_telemetry_definition("imu.cal_system", "System calibration", "IMU", "0-3", false);
    send_telemetry_definition("imu.cal_gyro", "Gyroscope calibration", "IMU", "0-3", false);
    send_telemetry_definition("imu.cal_accel", "Accelerometer calibration", "IMU", "0-3", false);
    send_telemetry_definition("imu.cal_mag", "Magnetometer calibration", "IMU", "0-3", false);

    send_telemetry_definition("flow.dx", "Frame X movement", "Optical flow", "counts");
    send_telemetry_definition("flow.dy", "Frame Y movement", "Optical flow", "counts");
    send_telemetry_definition("flow.total_x", "Accumulated X", "Optical flow", "counts");
    send_telemetry_definition("flow.total_y", "Accumulated Y", "Optical flow", "counts");

    send_telemetry_definition("inductive.detected", "Metal detected", "Inductive", "bool", false);
    send_telemetry_definition("inductive.count", "Detection count", "Inductive", "events");
    send_telemetry_definition("encoder.0", "Encoder 0 position", "Encoders", "counts");
    send_telemetry_definition("encoder.1", "Encoder 1 position", "Encoders", "counts");
    send_telemetry_definition("encoder.0_a", "Encoder 0 channel A", "Encoders", "bool", false);
    send_telemetry_definition("encoder.0_b", "Encoder 0 channel B", "Encoders", "bool", false);
    send_telemetry_definition("encoder.0_edges", "Encoder 0 valid edges", "Encoders", "edges");
    send_telemetry_definition("encoder.1_a", "Encoder 1 channel A", "Encoders", "bool", false);
    send_telemetry_definition("encoder.1_b", "Encoder 1 channel B", "Encoders", "bool", false);
    send_telemetry_definition("encoder.1_edges", "Encoder 1 valid edges", "Encoders", "edges");
    send_telemetry_definition("drive.left_percent", "Left drive command", "Drive", "%");
    send_telemetry_definition("drive.right_percent", "Right drive command", "Drive", "%");
    send_telemetry_definition("drive.left_us", "Left drive pulse", "Drive", "us");
    send_telemetry_definition("drive.right_us", "Right drive pulse", "Drive", "us");
    send_telemetry_definition("drive.active", "Drive active", "Drive", "bool", false);
    send_telemetry_definition("drum.left_percent", "Left drum command", "Drum", "%");
    send_telemetry_definition("drum.right_percent", "Right drum command", "Drum", "%");
    send_telemetry_definition("drum.left_us", "Left drum pulse", "Drum", "us");
    send_telemetry_definition("drum.right_us", "Right drum pulse", "Drum", "us");
    send_telemetry_definition("drum.active", "Drum active", "Drum", "bool", false);
    send_telemetry_definition("servo.pin", "Selected servo test pin", "Servo test", "D");
    send_telemetry_definition("servo.pulse_us", "Servo test pulse", "Servo test", "us");
    send_telemetry_definition("servo.active", "Servo test active", "Servo test", "bool", false);
    send_telemetry_definition("servo.angle_deg", "Servo target angle", "Servo", "deg");
    send_telemetry_definition("servo.position_mode", "Positional servo mode", "Servo", "bool", false);
    send_telemetry_definition("magnet.on", "Electromagnet energized", "Electromagnet", "bool", false);
    send_telemetry_definition("navigation.active", "Navigation active", "Navigation", "bool", false);
    send_telemetry_definition("navigation.state", "Navigation state", "Navigation", "state", false);
    send_telemetry_definition("navigation.stop_reason", "Navigation status detail", "Navigation", "text", false);
    send_telemetry_definition("navigation.front_mm", "Forward clearance", "Navigation", "mm");
    send_telemetry_definition("navigation.left_mm", "Left clearance", "Navigation", "mm");
    send_telemetry_definition("navigation.right_mm", "Right clearance", "Navigation", "mm");
    send_telemetry_definition("navigation.target_heading", "Turn target", "Navigation", "deg");
    send_telemetry_definition("explore.active", "Autonomous explore active", "Autonomous explore", "bool", false);
    send_telemetry_definition("explore.state", "Autonomous explore phase", "Autonomous explore", "state", false);
    send_telemetry_definition("explore.reason", "Autonomous explore status", "Autonomous explore", "text", false);
    send_telemetry_definition("explore.front_mm", "Coherent forward obstacle range", "Autonomous explore", "mm");
    send_telemetry_definition("explore.left_mm", "Left ultrasonic range", "Autonomous explore", "mm");
    send_telemetry_definition("explore.right_mm", "Right ultrasonic range", "Autonomous explore", "mm");
    send_telemetry_definition("explore.left_top_mm", "Left upper weight ToF", "Autonomous explore", "mm");
    send_telemetry_definition("explore.left_bottom_mm", "Left lower weight ToF", "Autonomous explore", "mm");
    send_telemetry_definition("explore.right_top_mm", "Right upper weight ToF", "Autonomous explore", "mm");
    send_telemetry_definition("explore.right_bottom_mm", "Right lower weight ToF", "Autonomous explore", "mm");
    send_telemetry_definition("explore.target_bearing_deg", "Detected weight bearing", "Autonomous explore", "deg");
    send_telemetry_definition("explore.weights_seen", "Weight acquisition attempts", "Autonomous explore", "count", false);
    send_telemetry_definition("explore.turns", "Obstacle avoidance turns", "Autonomous explore", "count", false);
    send_telemetry_definition("mission.ready", "Mission uploaded", "Mission", "bool", false);
    send_telemetry_definition("mission.active", "Robot-side mission active", "Mission", "bool", false);
    send_telemetry_definition("mission.state", "Mission phase", "Mission", "state", false);
    send_telemetry_definition("mission.reason", "Mission detail", "Mission", "text", false);
    send_telemetry_definition("mission.waypoint_index", "Current waypoint", "Mission", "index", false);
    send_telemetry_definition("mission.waypoint_count", "Uploaded waypoints", "Mission", "count", false);
    send_telemetry_definition("mission.pose_x_mm", "Mission X", "Mission", "mm");
    send_telemetry_definition("mission.pose_y_mm", "Mission Y", "Mission", "mm");
    send_telemetry_definition("mission.heading_deg", "Mission heading", "Mission", "deg");
    send_telemetry_definition("mission.front_mm", "Nearest raw forward range", "Mission", "mm");
    send_telemetry_definition("mission.left_mm", "Raw left ultrasonic range", "Mission", "mm");
    send_telemetry_definition("mission.right_mm", "Raw right ultrasonic range", "Mission", "mm");
    send_telemetry_definition("mission.front_edge_mm", "Forward hit to robot footprint", "Mission", "mm");
    send_telemetry_definition("mission.left_edge_mm", "Left hit to robot footprint", "Mission", "mm");
    send_telemetry_definition("mission.right_edge_mm", "Right hit to robot footprint", "Mission", "mm");
    send_telemetry_definition("mission.nearest_edge_mm", "Nearest sensor hit to robot footprint", "Mission", "mm");
    send_telemetry_definition("mission.nearest_sensor", "Sensor reporting nearest footprint hit", "Mission", "sensor", false);
    send_telemetry_definition("mission.detour_count", "Mission detours", "Mission", "count", false);
    send_telemetry_definition("bluetooth.active", "Bluetooth transport active", "Communications", "bool", false);
    send_telemetry_definition("bluetooth.rx_bytes", "Bluetooth raw bytes received", "Communications", "bytes");
    send_telemetry_definition("bluetooth.rx_lines", "Bluetooth newline frames received", "Communications", "lines");
    send_telemetry_definition("bluetooth.rx_messages", "Bluetooth messages received", "Communications", "messages");
    send_telemetry_definition("system.uptime_ms", "Robot uptime", "System", "ms");
}

static void send_definitions() {
    if (!jsonActive) {
        return;
    }

    send_telemetry_definitions();

    {
        JsonDocument doc;
        doc["type"] = "parameter_definition";
        doc["name"] = "telemetry.interval_ms";
        doc["label"] = "Telemetry interval";
        doc["description"] = "How often the robot sends a telemetry packet.";
        doc["datatype"] = "int";
        doc["value"] = (long)telemetryIntervalMs;
        doc["min"] = active_telemetry_minimum_ms();
        doc["max"] = TELEMETRY_INTERVAL_MAX_MS;
        doc["step"] = 10;
        doc["unit"] = "ms";
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "parameter_definition";
        doc["name"] = "navigation.speed_percent";
        doc["label"] = "Navigation forward speed";
        doc["description"] = "Autonomous forward command; capped at 60% until calibrated.";
        doc["datatype"] = "int";
        doc["value"] = navigation_get_speed_percent();
        doc["min"] = 5; doc["max"] = 60; doc["step"] = 5; doc["unit"] = "%";
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "parameter_definition";
        doc["name"] = "navigation.turn_percent";
        doc["label"] = "Navigation turn speed";
        doc["description"] = "Differential command used during obstacle turns.";
        doc["datatype"] = "int";
        doc["value"] = navigation_get_turn_percent();
        doc["min"] = 5; doc["max"] = 60; doc["step"] = 5; doc["unit"] = "%";
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "parameter_definition";
        doc["name"] = "navigation.front_stop_mm";
        doc["label"] = "Navigation obstacle distance";
        doc["description"] = "Turn when the centre of the 8x8 view is closer than this.";
        doc["datatype"] = "int";
        doc["value"] = navigation_get_front_stop_mm();
        doc["min"] = 100; doc["max"] = 1500; doc["step"] = 25; doc["unit"] = "mm";
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "parameter_definition";
        doc["name"] = "drive.max_percent";
        doc["label"] = "Drive speed limit";
        doc["description"] = "Maximum permitted keyboard/test drive command. Firmware hard-caps this at 100%.";
        doc["datatype"] = "int";
        doc["value"] = drive_control_get_max_percent();
        doc["min"] = 0;
        doc["max"] = DRIVE_HARD_MAX_PERCENT;
        doc["step"] = 5;
        doc["unit"] = "%";
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "drive_set";
        doc["label"] = "Drive motors";
        doc["description"] = "Left/right percent command. Requires Debug Mode; stops automatically after 300 ms without another command.";
        JsonArray args = doc["args"].to<JsonArray>();
        JsonObject left = args.add<JsonObject>();
        left["name"] = "left";
        left["label"] = "Left (%)";
        left["type"] = "int";
        left["min"] = -100;
        left["max"] = 100;
        left["step"] = 5;
        left["default"] = 0;
        JsonObject right = args.add<JsonObject>();
        right["name"] = "right";
        right["label"] = "Right (%)";
        right["type"] = "int";
        right["min"] = -100;
        right["max"] = 100;
        right["step"] = 5;
        right["default"] = 0;
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "drum_set";
        doc["label"] = "Drum motors";
        doc["description"] = "D28/D29 servo-style outputs; hold-to-run test, full +/-100% range and 300 ms timeout. Requires Debug Mode.";
        JsonArray args = doc["args"].to<JsonArray>();
        JsonObject left = args.add<JsonObject>();
        left["name"] = "left"; left["label"] = "Left (%)";
        left["type"] = "int"; left["min"] = -100; left["max"] = 100;
        left["step"] = 5; left["default"] = 0;
        JsonObject right = args.add<JsonObject>();
        right["name"] = "right"; right["label"] = "Right (%)";
        right["type"] = "int"; right["min"] = -100; right["max"] = 100;
        right["step"] = 5; right["default"] = 0;
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "servo_set";
        doc["label"] = "Servo pulse test · D20";
        doc["description"] = "Raw pulse diagnostic for the positional HX12K on D20. These values command positions, not motor speed. Prefer servo_angle_set; returns to 1500 us after 300 ms without refresh.";
        JsonArray args = doc["args"].to<JsonArray>();
        JsonObject pulse = args.add<JsonObject>();
        pulse["name"] = "pulse_us"; pulse["label"] = "Pulse width (us)";
        pulse["type"] = "int"; pulse["min"] = SERVO_TEST_MIN_US;
        pulse["max"] = SERVO_TEST_MAX_US; pulse["step"] = 25;
        pulse["default"] = SERVO_TEST_NEUTRAL_US;
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "servo_angle_set";
        doc["label"] = "Set servo angle · D20";
        doc["description"] = "Sets and holds the positional HX12K target from 0 to 180 degrees. Calibrated references: center 125, real weight 100, fake weight 150. Requires Debug Mode.";
        JsonObject angle = doc["args"].to<JsonArray>().add<JsonObject>();
        angle["name"] = "angle"; angle["label"] = "Target angle (degrees)";
        angle["type"] = "int"; angle["min"] = 0; angle["max"] = 180;
        angle["step"] = 1; angle["default"] = 90;
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "magnet_set";
        doc["label"] = "Electromagnet";
        doc["description"] = "Switch the D26 electromagnet driver. ON requires Debug Mode and a fresh app keepalive; STOP, Debug Mode exit, or a lost keepalive switches it OFF.";
        JsonObject arg = doc["args"].to<JsonArray>().add<JsonObject>();
        arg["name"] = "enabled";
        arg["label"] = "Energized";
        arg["type"] = "bool";
        arg["default"] = false;
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "navigation_set";
        doc["label"] = "Autonomous navigation";
        doc["description"] = "Start/stop conservative 8x8 TOF obstacle navigation. Start requires Debug Mode, valid IMU and valid forward range.";
        JsonObject arg = doc["args"].to<JsonArray>().add<JsonObject>();
        arg["name"] = "enabled";
        arg["label"] = "Enabled";
        arg["type"] = "bool";
        arg["default"] = false;
        send(doc);
    }
    for (const char* commandName : {"explore_start", "explore_stop"}) {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = commandName;
        doc["label"] = commandName;
        doc["description"] = "Arena-free autonomous exploration with drum and electromagnet held on throughout the run. Start requires Debug Mode, at least 80% drive limit and valid forward range data; STOP ends all outputs.";
        doc["args"].to<JsonArray>();
        send(doc);
    }
    // Mission upload is driven by the dedicated planner UI. These definitions
    // advertise the operator actions without exposing partial-upload internals.
    for (const char* commandName : {"mission_start", "mission_resume", "mission_stop"}) {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = commandName;
        doc["label"] = commandName;
        doc["description"] = "Robot-side mission control; upload the planner route before starting.";
        doc["args"].to<JsonArray>();
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "stop";
        doc["label"] = "STOP all motors";
        doc["description"] = "Always allowed, debug mode or not.";
        doc["args"].to<JsonArray>();
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "set_debug_mode";
        doc["label"] = "Set Debug Mode";
        doc["description"] = "Actuator commands are refused unless this is on.";
        JsonObject arg = doc["args"].to<JsonArray>().add<JsonObject>();
        arg["name"] = "enabled";
        arg["label"] = "Enabled";
        arg["type"] = "bool";
        arg["default"] = true;
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "encoders_reset";
        doc["label"] = "Reset Encoders";
        doc["description"] = "Zero both encoder counts before a measured run.";
        doc["args"].to<JsonArray>();
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "bluetooth_probe";
        doc["label"] = "Send Bluetooth Probe";
        doc["description"] = "Transmit one short diagnostic JSON line on Serial1 without moving hardware.";
        doc["args"].to<JsonArray>();
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "set_text_mode";
        doc["label"] = "Resume Text Output";
        doc["description"] = "Stop JSON telemetry and resume human-readable USB sensor output.";
        doc["args"].to<JsonArray>();
        send(doc);
    }

    send_state();
    lastDefinitionsMs = millis();
}

// =====================================================================
// Telemetry
// =====================================================================

// "tof_xshut0" -> "tof.xshut0". The GUI groups signals by the text before
// the first dot, so this gets us tof/ir/ultrasonic groupings for free.
static void dotted_name(const char* name, char* out, size_t outSize) {
    size_t i = 0;
    bool replaced = false;

    for (; name[i] != '\0' && i < outSize - 1; i++) {
        if (name[i] == '_' && !replaced) {
            out[i] = '.';
            replaced = true;
        } else {
            out[i] = name[i];
        }
    }
    out[i] = '\0';
}

void debug_protocol_send_telemetry() {
    if (!jsonActive) {
        return;
    }

    unsigned long now = millis();
    if (now - lastTelemetryMs < telemetryIntervalMs) {
        return;
    }
    lastTelemetryMs = now;

    JsonDocument doc;
    doc["type"] = "telemetry";
    doc["time"] = now;
    JsonObject data = doc["data"].to<JsonObject>();

    // Distance sensors (TOF, IR, ultrasonic) all share one interface. Every
    // key is included in every packet: null explicitly means invalid/offline,
    // preventing the GUI from leaving a stale old value on screen.
    char name[40];
    for (unsigned char i = 0; i < distance_sensors_count(); i++) {
        DistanceSensor* sensor = distance_sensor_get_by_index(i);
        if (sensor == nullptr) continue;
        dotted_name(sensor->getName(), name, sizeof(name));
        if (sensor->isValid()) data[name] = sensor->getDistanceMM();
        else data[name] = nullptr;
    }

    // These bring-up/status fields change infrequently; send them once per
    // second rather than bloating every large 8x8 telemetry packet.
    static unsigned long lastPointTofDiagnosticsMs = 0;
    if (now - lastPointTofDiagnosticsMs >= 1000) {
        lastPointTofDiagnosticsMs = now;
        data["tof.xshut_expander_ok"] = distance_sensors_xshut_expander_ready();
        for (uint8_t i = 0; i < distance_sensors_point_tof_count(); i++) {
            PointTofDiagnostic diagnostic;
            if (!distance_sensors_get_point_tof_diagnostic(i, &diagnostic)) continue;
            char prefix[40];
            char signal[64];
            dotted_name(diagnostic.name, prefix, sizeof(prefix));
            snprintf(signal, sizeof(signal), "%s.init_ok", prefix);
            data[signal] = diagnostic.initialized;
            snprintf(signal, sizeof(signal), "%s.range_status", prefix);
            data[signal] = diagnostic.rangeStatus;
            snprintf(signal, sizeof(signal), "%s.i2c_status", prefix);
            data[signal] = diagnostic.i2cStatus;
            snprintf(signal, sizeof(signal), "%s.model_id", prefix);
            char modelId[8];
            snprintf(modelId, sizeof(modelId), "0x%04X", diagnostic.modelId);
            data[signal] = modelId;
        }
    }

    uint16_t grid[64];
    const bool gridAvailable = distance_sensors_get_8x8_grid(grid);
    // A valid all-4000 frame means open space, not a failed sensor. Zone
    // values alone cannot distinguish those cases after invalid zones become
    // null in telemetry, so send frame health separately.
    data["tof.array_frame_ok"] = gridAvailable;
    uint16_t closest = 0;
    uint8_t validZones = 0;
    char zoneName[24];
    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t col = 0; col < 8; col++) {
            const uint8_t index = row * 8 + col;
            snprintf(zoneName, sizeof(zoneName), "tof.array.r%uc%u", row, col);
            if (gridAvailable && grid[index] > 0 && grid[index] < 4000) {
                data[zoneName] = grid[index];
                validZones++;
                if (closest == 0 || grid[index] < closest) closest = grid[index];
            } else {
                data[zoneName] = nullptr;
            }
    }
    }
    data["tof.array_valid_zones"] = validZones;
    if (closest > 0) data["tof.array_min"] = closest;
    else data["tof.array_min"] = nullptr;

    if (colour_is_valid()) {
        data["colour.r"] = colour_get_red();
        data["colour.g"] = colour_get_green();
        data["colour.b"] = colour_get_blue();
        data["colour.c"] = colour_get_clear();
    } else {
        data["colour.r"] = nullptr;
        data["colour.g"] = nullptr;
        data["colour.b"] = nullptr;
        data["colour.c"] = nullptr;
    }

    if (imu_is_valid()) {
        data["imu.heading"] = imu_get_heading();
        data["imu.roll"] = imu_get_roll();
        data["imu.pitch"] = imu_get_pitch();
        data["imu.cal_system"] = imu_get_system_calibration();
        data["imu.cal_gyro"] = imu_get_gyro_calibration();
        data["imu.cal_accel"] = imu_get_accel_calibration();
        data["imu.cal_mag"] = imu_get_mag_calibration();
    } else {
        data["imu.heading"] = nullptr;
        data["imu.roll"] = nullptr;
        data["imu.pitch"] = nullptr;
        data["imu.cal_system"] = nullptr;
        data["imu.cal_gyro"] = nullptr;
        data["imu.cal_accel"] = nullptr;
        data["imu.cal_mag"] = nullptr;
    }

    if (optical_flow_is_valid()) {
        data["flow.dx"] = optical_flow_get_delta_x();
        data["flow.dy"] = optical_flow_get_delta_y();
        data["flow.total_x"] = optical_flow_get_total_x();
        data["flow.total_y"] = optical_flow_get_total_y();
    } else {
        data["flow.dx"] = nullptr;
        data["flow.dy"] = nullptr;
        data["flow.total_x"] = nullptr;
        data["flow.total_y"] = nullptr;
    }

    data["inductive.detected"] = inductive_is_detected();
    data["inductive.count"] = (long)inductive_get_detection_count();

    data["encoder.0"] = encoder_get_position(0);
    data["encoder.1"] = encoder_get_position(1);
    data["encoder.0_a"] = encoder_get_channel_a(0);
    data["encoder.0_b"] = encoder_get_channel_b(0);
    data["encoder.0_edges"] = encoder_get_transition_count(0);
    data["encoder.1_a"] = encoder_get_channel_a(1);
    data["encoder.1_b"] = encoder_get_channel_b(1);
    data["encoder.1_edges"] = encoder_get_transition_count(1);

    data["drive.left_percent"] = drive_control_get_left_percent();
    data["drive.right_percent"] = drive_control_get_right_percent();
    data["drive.left_us"] = drive_control_get_left_microseconds();
    data["drive.right_us"] = drive_control_get_right_microseconds();
    data["drive.active"] = drive_control_is_active();
    data["drum.left_percent"] = drum_control_left_percent();
    data["drum.right_percent"] = drum_control_right_percent();
    data["drum.left_us"] = drum_control_left_us();
    data["drum.right_us"] = drum_control_right_us();
    data["drum.active"] = drum_control_is_active();
    data["servo.pin"] = servo_control_get_pin();
    data["servo.pulse_us"] = servo_control_get_microseconds();
    data["servo.active"] = servo_control_is_active();
    data["servo.angle_deg"] = servo_control_get_angle();
    data["servo.position_mode"] = servo_control_is_position_mode();
    data["magnet.on"] = magnet_control_is_on();
    data["navigation.active"] = navigation_is_active();
    data["navigation.state"] = navigation_get_state_name();
    data["navigation.stop_reason"] = navigation_get_stop_reason();
    if (navigation_get_front_mm()) data["navigation.front_mm"] = navigation_get_front_mm();
    else data["navigation.front_mm"] = nullptr;
    if (navigation_get_left_mm()) data["navigation.left_mm"] = navigation_get_left_mm();
    else data["navigation.left_mm"] = nullptr;
    if (navigation_get_right_mm()) data["navigation.right_mm"] = navigation_get_right_mm();
    else data["navigation.right_mm"] = nullptr;
    data["navigation.target_heading"] = navigation_get_target_heading();
    data["explore.active"] = exploration_mode_is_active();
    data["explore.state"] = exploration_mode_state();
    data["explore.reason"] = exploration_mode_reason();
    if (exploration_mode_front_mm()) data["explore.front_mm"] = exploration_mode_front_mm();
    else data["explore.front_mm"] = nullptr;
    if (exploration_mode_left_mm()) data["explore.left_mm"] = exploration_mode_left_mm();
    else data["explore.left_mm"] = nullptr;
    if (exploration_mode_right_mm()) data["explore.right_mm"] = exploration_mode_right_mm();
    else data["explore.right_mm"] = nullptr;
    if (exploration_mode_left_top_mm()) data["explore.left_top_mm"] = exploration_mode_left_top_mm();
    else data["explore.left_top_mm"] = nullptr;
    if (exploration_mode_left_bottom_mm()) data["explore.left_bottom_mm"] = exploration_mode_left_bottom_mm();
    else data["explore.left_bottom_mm"] = nullptr;
    if (exploration_mode_right_top_mm()) data["explore.right_top_mm"] = exploration_mode_right_top_mm();
    else data["explore.right_top_mm"] = nullptr;
    if (exploration_mode_right_bottom_mm()) data["explore.right_bottom_mm"] = exploration_mode_right_bottom_mm();
    else data["explore.right_bottom_mm"] = nullptr;
    data["explore.target_bearing_deg"] = exploration_mode_target_bearing_deg();
    data["explore.weights_seen"] = exploration_mode_weights_seen();
    data["explore.turns"] = exploration_mode_turns();
    data["mission.ready"] = mission_is_ready();
    data["mission.active"] = mission_is_active();
    data["mission.state"] = mission_state();
    data["mission.reason"] = mission_reason();
    data["mission.waypoint_index"] = mission_waypoint_index();
    data["mission.waypoint_count"] = mission_waypoint_count();
    data["mission.pose_x_mm"] = mission_x_mm();
    data["mission.pose_y_mm"] = mission_y_mm();
    data["mission.heading_deg"] = mission_heading_deg();
    if (mission_front_mm()) data["mission.front_mm"] = mission_front_mm();
    else data["mission.front_mm"] = nullptr;
    if (mission_left_mm()) data["mission.left_mm"] = mission_left_mm();
    else data["mission.left_mm"] = nullptr;
    if (mission_right_mm()) data["mission.right_mm"] = mission_right_mm();
    else data["mission.right_mm"] = nullptr;
    const int16_t frontEdge = mission_front_edge_clearance_mm();
    const int16_t leftEdge = mission_left_edge_clearance_mm();
    const int16_t rightEdge = mission_right_edge_clearance_mm();
    const int16_t nearestEdge = mission_nearest_edge_clearance_mm();
    if (frontEdge != -32768) data["mission.front_edge_mm"] = frontEdge;
    else data["mission.front_edge_mm"] = nullptr;
    if (leftEdge != -32768) data["mission.left_edge_mm"] = leftEdge;
    else data["mission.left_edge_mm"] = nullptr;
    if (rightEdge != -32768) data["mission.right_edge_mm"] = rightEdge;
    else data["mission.right_edge_mm"] = nullptr;
    if (nearestEdge != -32768) data["mission.nearest_edge_mm"] = nearestEdge;
    else data["mission.nearest_edge_mm"] = nullptr;
    if (mission_nearest_sensor()[0]) data["mission.nearest_sensor"] = mission_nearest_sensor();
    else data["mission.nearest_sensor"] = nullptr;
    data["mission.detour_count"] = mission_detour_count();
    data["bluetooth.active"] = jsonActive && activeTransport == DebugTransport::Bluetooth;
    data["bluetooth.rx_bytes"] = (long)console_bluetooth_rx_bytes();
    data["bluetooth.rx_lines"] = (long)console_bluetooth_rx_lines();
    data["bluetooth.rx_messages"] = (long)bluetoothRxMessages;
    data["system.uptime_ms"] = now;

    send(doc);
}

// =====================================================================
// Receiving
// =====================================================================

void debug_protocol_set_active(bool active) {
    if (jsonActive == active) {
        return;
    }
    jsonActive = active;

    if (modeChangedHandler != nullptr) {
        modeChangedHandler(active);
    }

    if (!active) {
        protocolOutput->println("debug protocol: JSON mode off; text output resumed");
    }
}

static void handle_command(JsonDocument& doc) {
    const char* command = doc["command"] | "";

    if (strcmp(command, "stop") == 0) {
        // Deliberately always allowed - a stop must never be gated.
        drive_control_stop();
        drum_control_stop();
        magnet_control_off();
        servo_control_stop();
        exploration_mode_stop("Stopped: emergency stop");
        navigation_stop("Emergency stop");
        mission_stop("Emergency stop");
        debug_protocol_log("WARNING", "STOP: actuator outputs set to safe state");
        send_state();

    } else if (strcmp(command, "set_debug_mode") == 0) {
        debugMode = doc["enabled"] | false;
        debug_protocol_log("INFO", debugMode ? "Debug mode enabled"
                                             : "Debug mode disabled");
        if (!debugMode) {
            drive_control_stop();
            drum_control_stop();
            magnet_control_off();
            servo_control_stop();
            exploration_mode_stop("Stopped: debug mode disabled");
            navigation_stop("Debug mode disabled");
            mission_stop("Debug mode disabled");
        }
        send_state();

    } else if (strcmp(command, "servo_angle_set") == 0) {
        if (!debugMode) {
            send_error("servo_angle_set requires debug mode");
            return;
        }
        servo_control_set_angle(doc["angle"] | 90);

    } else if (strcmp(command, "servo_set") == 0) {
        if (!debugMode) {
            servo_control_stop();
            send_error("servo_set requires debug mode");
            return;
        }
        const int pulseUs = doc["pulse_us"] | SERVO_TEST_NEUTRAL_US;
        servo_control_set_microseconds(pulseUs);

    } else if (strcmp(command, "drive_set") == 0) {
        if (!debugMode) {
            send_error("drive_set requires debug mode");
            return;
        }
        exploration_mode_stop("Stopped: manual drive command took control");
        navigation_stop("Manual drive command");
        mission_stop("Manual drive command");
        drive_control_set_percent(doc["left"] | 0, doc["right"] | 0);

    } else if (strcmp(command, "drum_set") == 0) {
        if (!debugMode) {
            send_error("drum_set requires debug mode");
            return;
        }
        exploration_mode_stop("Stopped: manual drum command took control");
        const int left = doc["left"] | 0;
        const int right = doc["right"] | 0;
        drum_control_set_percent(left, right);

    } else if (strcmp(command, "magnet_set") == 0) {
        const bool enabled = doc["enabled"] | false;
        // An OFF command is always accepted; energizing the coil requires
        // Debug Mode. The GUI repeats ON while its connection is healthy.
        if (enabled && !debugMode) {
            send_error("magnet_set ON requires debug mode");
            return;
        }
        magnet_control_set(enabled);

    } else if (strcmp(command, "navigation_set") == 0) {
        const bool enabled = doc["enabled"] | false;
        if (enabled && !debugMode) {
            send_error("navigation_set requires debug mode");
            return;
        }
        if (enabled) mission_stop("8x8 test navigation took control");
        if (enabled) exploration_mode_stop("Stopped: legacy navigation took control");
        if (!navigation_set_enabled(enabled)) {
            send_error(navigation_get_stop_reason());
        } else {
            debug_protocol_log("INFO", enabled ? "Navigation started" : "Navigation stopped");
        }

    } else if (strcmp(command, "explore_start") == 0) {
        if (!debugMode) {
            send_error("explore_start requires debug mode");
            return;
        }
        navigation_stop("Arena-free explore took control");
        mission_stop("Arena-free explore took control");
        if (!exploration_mode_start()) {
            send_error(exploration_mode_reason());
        } else {
            debug_protocol_log("INFO", "Arena-free autonomous exploration started");
        }
    } else if (strcmp(command, "explore_stop") == 0) {
        exploration_mode_stop("Stopped by operator");
        debug_protocol_log("INFO", "Autonomous exploration stopped");
    } else if (strcmp(command, "explore_keepalive") == 0) {
        if (debugMode) exploration_mode_keepalive();

    } else if (strcmp(command, "mission_begin") == 0) {
        exploration_mode_stop("Stopped: arena mission configuration took control");
        const int count = doc["count"] | 0;
        if (!debugMode || count < 1 || count > MISSION_MAX_WAYPOINTS ||
            !mission_begin(static_cast<uint8_t>(count), doc["start_x_mm"] | -1.0f,
                           doc["start_y_mm"] | -1.0f,
                           doc["start_heading_deg"] | 0.0f,
                           doc["robot_radius_mm"] | 215.0f,
                           doc["margin_mm"] | 90.0f,
                           doc["encoder0_mm_per_count"] | ENCODER0_MM_PER_COUNT,
                           doc["encoder1_mm_per_count"] | ENCODER1_MM_PER_COUNT,
                           doc["encoder0_reversed"] | static_cast<bool>(ENCODER0_REVERSED),
                           doc["encoder1_reversed"] | static_cast<bool>(ENCODER1_REVERSED)))
            send_error("Mission begin rejected: check Debug Mode, start, route size and encoder scales");
    } else if (strcmp(command, "mission_sensor") == 0) {
        const char* key = doc["key"] | "";
        if (!debugMode || !mission_set_sensor(
                key, doc["x_mm"] | 0.0f, doc["y_mm"] | 0.0f,
                doc["angle_deg"] | 0.0f, doc["height_mm"] | 0.0f,
                doc["enabled"] | true, doc["matrix_fov_deg"] | 60.0f,
                doc["matrix_mirrored"] | false))
            send_error("Mission sensor rejected: unknown key or invalid mounting geometry");
    } else if (strcmp(command, "mission_obstacle") == 0) {
        const int index = doc["index"] | -1;
        if (!debugMode || index < 0 || index >= MISSION_MAX_OBSTACLES ||
            !mission_add_obstacle(static_cast<uint8_t>(index),
                                  doc["x"] | -1.0f, doc["y"] | -1.0f,
                                  doc["width"] | 0.0f, doc["depth"] | 0.0f,
                                  doc["rotation"] | 0.0f,
                                  doc["circular"] | false))
            send_error("Mission obstacle rejected: sequence or geometry invalid");
    } else if (strcmp(command, "mission_chunk") == 0) {
        const int offset = doc["offset"] | -1;
        JsonArray points = doc["points"].as<JsonArray>();
        if (!debugMode || offset < 0 || points.isNull() || points.size() == 0 ||
            points.size() % 3 != 0 || points.size() > 24 ||
            offset + static_cast<int>(points.size()/3) > MISSION_MAX_WAYPOINTS) {
            send_error("Mission waypoint chunk malformed");
            return;
        }
        for (size_t i = 0; i < points.size(); i += 3) {
            const int flags = points[i+2].as<int>();
            if (flags < 0 || flags > 255 ||
                !mission_add_waypoint(static_cast<uint8_t>(offset + i/3),
                                      points[i].as<float>(), points[i+1].as<float>(),
                                      static_cast<uint8_t>(flags))) {
                send_error("Mission waypoint rejected: sequence or coordinates invalid");
                return;
            }
        }
    } else if (strcmp(command, "mission_commit") == 0) {
        if (!debugMode || !mission_commit()) send_error("Mission commit rejected: incomplete or unsafe route/map");
        else debug_protocol_log("INFO", "Robot-side route uploaded and validated; awaiting Start");
    } else if (strcmp(command, "mission_start") == 0) {
        exploration_mode_stop("Stopped: arena mission took control");
        if (!debugMode || !mission_start()) send_error("Mission start rejected: upload route, calibrate IMU and set 100% drive limit");
        else debug_protocol_log("INFO", "Robot-side mission started");
    } else if (strcmp(command, "mission_resume") == 0) {
        exploration_mode_stop("Stopped: arena mission took control");
        if (!debugMode || !mission_resume()) send_error("Mission resume rejected: not paused at weight site");
    } else if (strcmp(command, "mission_stop") == 0) {
        exploration_mode_stop("Stopped: arena mission stopped");
        mission_stop("Stopped by operator");
    } else if (strcmp(command, "encoders_reset") == 0) {
        encoders_reset();
        debug_protocol_log("INFO", "Encoder counts reset");

    } else if (strcmp(command, "bluetooth_probe") == 0) {
        // Deliberately short enough to fit in the hardware UART's immediate
        // transmit capacity. This cannot move hardware and is useful even if
        // the far side of the radio is disconnected.
        BLUETOOTH_PORT.print("{\"type\":\"bluetooth_probe\"}\n");
        debug_protocol_log("INFO", "Bluetooth Serial1 probe transmitted");

    } else if (strcmp(command, "set_text_mode") == 0) {
        debug_protocol_log("INFO", "Leaving JSON mode");
        debug_protocol_set_active(false);

    } else {
        send_error("Unknown command");
    }
}

static void handle_parameter(JsonDocument& doc) {
    const char* name = doc["name"] | "";

    if (strcmp(name, "telemetry.interval_ms") == 0) {
        long requested = doc["value"] | (long)telemetryIntervalMs;
        telemetryIntervalMs = constrain(requested,
                                        (long)active_telemetry_minimum_ms(),
                                        (long)TELEMETRY_INTERVAL_MAX_MS);
        // Echo the accepted value, not the requested one - the GUI needs to
        // see the clamped result so the two ends don't drift apart.
        send_parameter_value("telemetry.interval_ms", (long)telemetryIntervalMs);

    } else if (strcmp(name, "servo.pulse_us") == 0) {
        if (!debugMode) {
            send_error("servo.pulse_us requires debug mode");
            return;
        }
        servo_control_set_microseconds(doc["value"] | SERVO_TEST_NEUTRAL_US);
        send_parameter_value("servo.pulse_us", servo_control_get_microseconds());

    } else if (strcmp(name, "drive.max_percent") == 0) {
        if (!debugMode) {
            send_error("drive.max_percent requires debug mode");
            return;
        }
        drive_control_set_max_percent(doc["value"] | DRIVE_DEFAULT_MAX_PERCENT);
        send_parameter_value("drive.max_percent", drive_control_get_max_percent());

    } else if (strcmp(name, "navigation.speed_percent") == 0) {
        navigation_set_speed_percent(doc["value"] | 30);
        send_parameter_value(name, navigation_get_speed_percent());
    } else if (strcmp(name, "navigation.turn_percent") == 0) {
        navigation_set_turn_percent(doc["value"] | 25);
        send_parameter_value(name, navigation_get_turn_percent());
    } else if (strcmp(name, "navigation.front_stop_mm") == 0) {
        navigation_set_front_stop_mm(doc["value"] | 300);
        send_parameter_value(name, navigation_get_front_stop_mm());

    } else {
        send_error("Unknown parameter");
    }
}

static void handle_parameter_request(JsonDocument& doc) {
    const char* name = doc["name"] | "";

    if (strcmp(name, "telemetry.interval_ms") == 0) {
        send_parameter_value("telemetry.interval_ms", (long)telemetryIntervalMs);
    } else if (strcmp(name, "servo.pulse_us") == 0) {
        send_parameter_value("servo.pulse_us", servo_control_get_microseconds());
    } else if (strcmp(name, "drive.max_percent") == 0) {
        send_parameter_value("drive.max_percent", drive_control_get_max_percent());
    } else if (strcmp(name, "navigation.speed_percent") == 0) {
        send_parameter_value(name, navigation_get_speed_percent());
    } else if (strcmp(name, "navigation.turn_percent") == 0) {
        send_parameter_value(name, navigation_get_turn_percent());
    } else if (strcmp(name, "navigation.front_stop_mm") == 0) {
        send_parameter_value(name, navigation_get_front_stop_mm());
    } else {
        send_error("Unknown parameter");
    }
}

static void handle_json_from(const char* json, DebugTransport transport) {
    // A newly received JSON message owns subsequent replies. This permits the
    // same GUI to connect through either direct USB or the CH9143 COM port.
    select_transport(transport);

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);

    if (error) {
        // Only meaningful to complain once we're actually talking JSON.
        if (jsonActive) {
            send_error("Malformed JSON");
        }
        return;
    }

    const char* type = doc["type"] | "";

    if (strcmp(type, "hello") == 0) {
        // The GUI connecting is what flips us into JSON mode.
        debug_protocol_set_active(true);
        debug_protocol_log("INFO", transport == DebugTransport::Bluetooth
            ? "Debug GUI connected over Bluetooth Serial1"
            : "Debug GUI connected over USB Serial");
        // The proven CH9143 workflow performs the entire handshake from one
        // hello. This avoids two immediate catalogue broadcasts competing on
        // the half-duplex radio link.
        send_definitions();

    } else if (strcmp(type, "request_definitions") == 0) {
        debug_protocol_set_active(true);
        if (millis() - lastDefinitionsMs >= 500) {
            send_definitions();
        }

    } else if (strcmp(type, "command") == 0) {
        handle_command(doc);

    } else if (strcmp(type, "parameter") == 0) {
        handle_parameter(doc);

    } else if (strcmp(type, "parameter_request") == 0) {
        handle_parameter_request(doc);

    } else {
        send_error("Unknown message type");
    }
}

void debug_protocol_handle_json(const char* json) {
    handle_json_from(json, DebugTransport::Usb);
}

void debug_protocol_handle_bluetooth_json(const char* json) {
    bluetoothRxMessages++;
    handle_json_from(json, DebugTransport::Bluetooth);
}
