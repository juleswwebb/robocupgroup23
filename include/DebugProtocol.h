//************************************
//         DebugProtocol.h
//************************************
//
// Newline-delimited JSON protocol for the Python debug console in
// tools/debug_gui. Full spec: docs/communicationProtocol.md.
//
// The firmware normally prints human-readable text ("tof_xshut0: 163 mm").
// When the GUI connects it sends {"type":"hello"}, and receiving that
// switches us into JSON mode: text prints stop and grouped telemetry
// packets start. "set_text_mode" (or the console's "mode sensors") goes
// back, so the plain serial monitor stays usable.
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

// Feed one received line that looks like JSON (starts with '{').
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
