//************************************
//         DebugProtocol.h
//************************************
//
// Newline-delimited JSON protocol for the Python debug console in
// tools/debug_gui. Full spec: docs/communicationProtocol.md.
//
// The JSON protocol uses Teensy USB Serial. When the GUI connects it sends
// {"type":"hello"}; receiving that starts grouped JSON telemetry and silences
// the human-readable sensor print tasks so the two formats cannot interleave.
//
// Actuator commands are gated behind debug mode, per the protocol doc's
// safety guidance - the firmware is the final authority on what's safe to
// run, not the GUI.

#ifndef DEBUG_PROTOCOL_H_
#define DEBUG_PROTOCOL_H_

// Notifies main.cpp when we enter/leave JSON mode, so it can silence the
// human-readable print tasks (which would otherwise interleave garbage
// into the JSON stream).
typedef void (*DebugModeChangedHandler)(bool json_active);
void debug_protocol_set_mode_changed_handler(DebugModeChangedHandler handler);

void debug_protocol_init();

// Send telemetry when its configured interval has elapsed. Incoming lines are
// collected by Console and routed to debug_protocol_handle_json().
void debug_protocol_update();

// Process one complete JSON line routed here by Console.
void debug_protocol_handle_json(const char* json);

// Send one grouped telemetry packet. Call periodically from a scheduled
// task; does nothing unless JSON mode is active.
void debug_protocol_send_telemetry();

bool debug_protocol_is_active();
void debug_protocol_set_active(bool active);

// Structured log line to the GUI's Logs tab. Levels: DEBUG/INFO/WARNING/
// ERROR/CRITICAL. Falls back to plain text when JSON mode is off.
void debug_protocol_log(const char* level, const char* message);

#endif /* DEBUG_PROTOCOL_H_ */
