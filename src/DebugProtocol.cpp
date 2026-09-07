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
#include <string.h>

// Where the debug console is connected. USB serial for now; swapping this
// to a hardware UART is all that's needed for the CH9143 Bluetooth pair,
// since the protocol itself is transport-independent.
#define DEBUG_SERIAL Serial

#define TELEMETRY_INTERVAL_MIN_MS 20
#define TELEMETRY_INTERVAL_MAX_MS 2000

static bool jsonActive = false;
static bool debugMode = false;
static unsigned long telemetryIntervalMs = 50; // 20 Hz, per the protocol doc
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

// ---------------------------------------------------------------------
// Definitions - these are what let the GUI build its own controls, so
// adding a tunable or a test routine never needs a Python change.
// ---------------------------------------------------------------------

static void send_definitions() {
    if (!jsonActive) {
        return;
    }

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
        doc["label"] = "Back to Text Output";
        doc["description"] = "Leave JSON mode so a plain serial monitor is readable again.";
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

    // Distance sensors (TOF, IR, ultrasonic) all share one interface, so
    // they can be walked generically. Invalid readings are left out
    // entirely rather than reported as a misleading zero.
    char name[40];
    for (unsigned char i = 0; i < distance_sensors_count(); i++) {
        DistanceSensor* sensor = distance_sensor_get_by_index(i);
        if (sensor == nullptr || !sensor->isValid()) {
            continue;
        }
        dotted_name(sensor->getName(), name, sizeof(name));
        data[name] = sensor->getDistanceMM();
    }

    unsigned short closest = distance_sensors_8x8_min_mm();
    if (closest > 0) {
        data["tof.array_min"] = closest;
    }

    if (colour_is_valid()) {
        data["colour.r"] = colour_get_red();
        data["colour.g"] = colour_get_green();
        data["colour.b"] = colour_get_blue();
        data["colour.c"] = colour_get_clear();
    }

    if (imu_is_valid()) {
        data["imu.heading"] = imu_get_heading();
        data["imu.roll"] = imu_get_roll();
        data["imu.pitch"] = imu_get_pitch();
        data["imu.cal_system"] = imu_get_system_calibration();
        data["imu.cal_gyro"] = imu_get_gyro_calibration();
        data["imu.cal_accel"] = imu_get_accel_calibration();
        data["imu.cal_mag"] = imu_get_mag_calibration();
    }

    if (optical_flow_is_valid()) {
        data["flow.dx"] = optical_flow_get_delta_x();
        data["flow.dy"] = optical_flow_get_delta_y();
        data["flow.total_x"] = optical_flow_get_total_x();
        data["flow.total_y"] = optical_flow_get_total_y();
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
        DEBUG_SERIAL.println("debug protocol: JSON mode off, text output resumed");
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
