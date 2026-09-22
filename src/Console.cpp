#include "Console.h"
#include "sensor_config.h"
#include <Arduino.h>
#include <string.h>
#include <ctype.h>

// Sized for the debug console's JSON messages, which are far longer than
// the hand-typed text commands (a command with several arguments can run
// well past a hundred characters).
#define CONSOLE_LINE_BUFFER_SIZE 256

struct ConsoleInput {
    char buffer[CONSOLE_LINE_BUFFER_SIZE];
    uint8_t length;
};

static ConsoleInput usbInput = {{0}, 0};
#if BLUETOOTH_ENABLED
static ConsoleInput bluetoothInput = {{0}, 0};
#endif
static ConsoleCommandHandler commandHandler = nullptr;
static ConsoleJsonHandler jsonHandler = nullptr;
static ConsoleJsonHandler bluetoothJsonHandler = nullptr;
static Print* currentOutput = &Serial;

void console_set_command_handler(ConsoleCommandHandler handler) {
    commandHandler = handler;
}

void console_set_json_handler(ConsoleJsonHandler handler) {
    jsonHandler = handler;
}

void console_set_bluetooth_json_handler(ConsoleJsonHandler handler) {
    bluetoothJsonHandler = handler;
}

Print& console_output() {
    return *currentOutput;
}

void console_init() {
    usbInput.length = 0;
#if BLUETOOTH_ENABLED
    bluetoothInput.length = 0;
#endif
}

static void dispatchLine(char* line, Print& output, ConsoleJsonHandler sourceJsonHandler) {
    // Split off the first whitespace-separated token as the command;
    // everything after it (trimmed) is passed through as args.
    char* command = line;
    while (*command == ' ') command++;

    // JSON goes to the debug protocol untouched - the lowercasing below
    // would otherwise corrupt string values inside the message.
    if (*command == '{') {
        if (sourceJsonHandler) {
            sourceJsonHandler(command);
        }
        return;
    }

    char* args = command;
    while (*args && *args != ' ') {
        *args = (char)tolower(*args);
        args++;
    }
    if (*args == ' ') {
        *args = '\0';
        args++;
        while (*args == ' ') args++;
    }

    if (*command == '\0') {
        return; // blank line
    }

    if (commandHandler) {
        currentOutput = &output;
        commandHandler(command, args);
        currentOutput = &Serial;
    }
}

static void pollInput(Stream& input, Print& output, ConsoleInput& state,
                      ConsoleJsonHandler sourceJsonHandler) {
    while (input.available()) {
        char c = (char)input.read();

        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            state.buffer[state.length] = '\0';
            dispatchLine(state.buffer, output, sourceJsonHandler);
            state.length = 0;
            continue;
        }

        if (state.length < CONSOLE_LINE_BUFFER_SIZE - 1) {
            state.buffer[state.length++] = c;
        }
        // else: silently drop overflow chars, line will still dispatch on '\n'
    }
}

void console_update() {
    pollInput(Serial, Serial, usbInput, jsonHandler);
#if BLUETOOTH_ENABLED
    pollInput(BLUETOOTH_PORT, BLUETOOTH_PORT, bluetoothInput,
              bluetoothJsonHandler);
#endif
}
