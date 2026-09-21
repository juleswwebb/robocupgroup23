#include "DebugProtocol.h"
#include "DistanceSensors.h"
#include "OpticalFlow.h"
#include "IMU.h"
#include "Inductive.h"
#include "Encoders.h"
#include "ServoControl.h"
#include "sensors.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdio.h>
#include <string.h>

// The desktop app and the text console share Teensy USB Serial. Console.cpp
// owns line buffering and routes JSON lines here; entering JSON mode disables
// the human-readable print tasks so protocol messages remain parseable.
#define DEBUG_SERIAL Serial

#define TELEMETRY_INTERVAL_MIN_MS 20
#define TELEMETRY_INTERVAL_MAX_MS 2000

static bool jsonActive = false;
static bool debugMode = false;
static unsigned long telemetryIntervalMs = 100; // 10 Hz; USB CDC has ample room for the 8x8 frame
static unsigned long lastTelemetryMs = 0;
static DebugModeChangedHandler modeChangedHandler = nullptr;

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
    serializeJson(doc, DEBUG_SERIAL);
    DEBUG_SERIAL.println();
}

void debug_protocol_log(const char* level, const char* message) {
    if (!jsonActive) {
        DEBUG_SERIAL.print("[");
        DEBUG_SERIAL.print(level);
        DEBUG_SERIAL.print("] ");
        DEBUG_SERIAL.println(message);
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
    doc["stopped"] = (servo_control_get_microseconds() == 1500);
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

    send_telemetry_definition("tof.array_min", "8x8 nearest valid zone", "8x8 TOF", "mm");
    send_telemetry_definition("tof.array_valid_zones", "8x8 valid zones", "8x8 TOF", "zones", false);

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
    send_telemetry_definition("servo.us", "Servo command", "Actuators", "us");
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
        doc["min"] = TELEMETRY_INTERVAL_MIN_MS;
        doc["max"] = TELEMETRY_INTERVAL_MAX_MS;
        doc["step"] = 10;
        doc["unit"] = "ms";
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "parameter_definition";
        doc["name"] = "servo.pulse_us";
        doc["label"] = "Servo pulse width";
        doc["description"] = "1000 = full reverse, 1500 = stop, 2000 = full forward.";
        doc["datatype"] = "int";
        doc["value"] = servo_control_get_microseconds();
        doc["min"] = 1000;
        doc["max"] = 2000;
        doc["step"] = 10;
        doc["unit"] = "us";
        send(doc);
    }
    {
        JsonDocument doc;
        doc["type"] = "command_definition";
        doc["name"] = "stop";
        doc["label"] = "STOP (servo neutral)";
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
        doc["name"] = "servo_set";
        doc["label"] = "Servo Test";
        doc["description"] = "Drive the servo. Requires debug mode.";
        JsonArray args = doc["args"].to<JsonArray>();

        JsonObject speed = args.add<JsonObject>();
        speed["name"] = "speed";
        speed["label"] = "Speed (%)";
        speed["type"] = "int";
        speed["min"] = -100;
        speed["max"] = 100;
        speed["step"] = 5;
        speed["default"] = 0;

        JsonObject us = args.add<JsonObject>();
        us["name"] = "us";
        us["label"] = "Pulse (us, 0 = use speed)";
        us["type"] = "int";
        us["min"] = 0;
        us["max"] = 2500;
        us["step"] = 10;
        us["default"] = 0;
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
        doc["name"] = "set_text_mode";
        doc["label"] = "Resume Text Output";
        doc["description"] = "Stop JSON telemetry and resume human-readable USB sensor output.";
        doc["args"].to<JsonArray>();
        send(doc);
    }

    send_state();
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

    uint16_t grid[64];
    const bool gridAvailable = distance_sensors_get_8x8_grid(grid);
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

    data["servo.us"] = servo_control_get_microseconds();
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
        DEBUG_SERIAL.println("debug protocol: JSON mode off; text output resumed");
    }
}

static void handle_command(JsonDocument& doc) {
    const char* command = doc["command"] | "";

    if (strcmp(command, "stop") == 0) {
        // Deliberately always allowed - a stop must never be gated.
        servo_control_set_speed(0);
        debug_protocol_log("WARNING", "STOP: servo set to neutral");
        send_state();

    } else if (strcmp(command, "set_debug_mode") == 0) {
        debugMode = doc["enabled"] | false;
        debug_protocol_log("INFO", debugMode ? "Debug mode enabled"
                                             : "Debug mode disabled");
        if (!debugMode) {
            servo_control_set_speed(0); // don't leave an actuator running
        }
        send_state();

    } else if (strcmp(command, "servo_set") == 0) {
        if (!debugMode) {
            send_error("servo_set requires debug mode");
            return;
        }
        // us wins when given; otherwise fall back to the speed percentage.
        int us = doc["us"] | 0;
        if (us > 0) {
            servo_control_set_microseconds(us);
        } else if (doc["angle"].is<int>()) {
            servo_control_set_angle(doc["angle"] | 90);
        } else {
            servo_control_set_speed(doc["speed"] | 0);
        }
        send_parameter_value("servo.pulse_us", servo_control_get_microseconds());

    } else if (strcmp(command, "encoders_reset") == 0) {
        encoders_reset();
        debug_protocol_log("INFO", "Encoder counts reset");

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
                                        (long)TELEMETRY_INTERVAL_MIN_MS,
                                        (long)TELEMETRY_INTERVAL_MAX_MS);
        // Echo the accepted value, not the requested one - the GUI needs to
        // see the clamped result so the two ends don't drift apart.
        send_parameter_value("telemetry.interval_ms", (long)telemetryIntervalMs);

    } else if (strcmp(name, "servo.pulse_us") == 0) {
        if (!debugMode) {
            send_error("servo.pulse_us requires debug mode");
            return;
        }
        servo_control_set_microseconds(doc["value"] | 1500);
        send_parameter_value("servo.pulse_us", servo_control_get_microseconds());

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
    } else {
        send_error("Unknown parameter");
    }
}

void debug_protocol_handle_json(const char* json) {
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
        debug_protocol_log("INFO", "Debug GUI connected");
        send_state();

    } else if (strcmp(type, "request_definitions") == 0) {
        debug_protocol_set_active(true);
        send_definitions();

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
