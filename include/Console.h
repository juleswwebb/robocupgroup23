//************************************
//         Console.h
//************************************
//
// Reads newline-terminated commands from Serial (non-blocking - just call
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

// command: the first whitespace-separated token, lowercased.
// args: everything after the command (may be empty string, never NULL).
typedef void (*ConsoleCommandHandler)(const char* command, const char* args);

void console_set_command_handler(ConsoleCommandHandler handler);

// Call once from setup(), after Serial.begin().
void console_init();

// Call every scheduler tick - cheap no-op when there's no serial input waiting.
void console_update();

#endif /* CONSOLE_H_ */
