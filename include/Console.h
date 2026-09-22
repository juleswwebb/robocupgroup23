//************************************
//         Console.h
//************************************
//
// Reads newline-terminated commands from USB Serial and, when enabled,
// Serial1 Bluetooth (non-blocking - just call
// console_update() every scheduler tick) and dispatches them to whatever
// handler main.cpp registers. Console itself knows nothing about sensors,
// modes, or the servo; it's just a generic line reader + dispatcher, so
// main.cpp (which owns all the Task objects that a "test mode" needs to
// enable/disable) can decide what each command actually does.
//
// Command format: "<command> <args...>", e.g. "servo speed 50", "mode test".
// Type "help" for the current command list.

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <Arduino.h>

// command: the first whitespace-separated token, lowercased.
// args: everything after the command (may be empty string, never NULL).
typedef void (*ConsoleCommandHandler)(const char* command, const char* args);

void console_set_command_handler(ConsoleCommandHandler handler);

// Lines starting with '{' are passed here untouched instead of being
// lowercased and tokenised as a text command - that's the debug console's
// JSON protocol (see DebugProtocol.h).
typedef void (*ConsoleJsonHandler)(const char* json);

void console_set_json_handler(ConsoleJsonHandler handler);
void console_set_bluetooth_json_handler(ConsoleJsonHandler handler);

// Output stream associated with the command currently being dispatched.
// This lets typed commands reply over the same USB/Bluetooth link they arrived
// on. Only use it synchronously from the registered command handler.
Print& console_output();

// Call once from setup(), after Serial.begin().
void console_init();

// Call every scheduler tick - cheap no-op when there's no serial input waiting.
void console_update();

// Raw Serial1 diagnostics. These count bytes before JSON parsing, allowing a
// broken radio/wiring path to be distinguished from malformed messages.
uint32_t console_bluetooth_rx_bytes();
uint32_t console_bluetooth_rx_lines();

#endif /* CONSOLE_H_ */
