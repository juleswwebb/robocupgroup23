#include "Console.h"
#include "sensor_config.h"
#include <Arduino.h>
#include <string.h>
#include <ctype.h>

// Sized for the debug console's JSON messages, which are far longer than
// the hand-typed text commands (a command with several arguments can run
// well past a hundred characters).
#define CONSOLE_LINE_BUFFER_SIZE 768

struct ConsoleInput {
    char buffer[CONSOLE_LINE_BUFFER_SIZE];
    size_t length;
    bool droppingLine;
};

static ConsoleInput usbInput = {{0}, 0, false};
#if BLUETOOTH_ENABLED
static ConsoleInput bluetoothInput = {{0}, 0, false};
#endif
static ConsoleCommandHandler commandHandler = nullptr;
static ConsoleJsonHandler jsonHandler = nullptr;
static ConsoleJsonHandler bluetoothJsonHandler = nullptr;
static Print* currentOutput = &Serial;
static uint32_t bluetoothRxBytes = 0;
static uint32_t bluetoothRxLines = 0;

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
    usbInput.droppingLine = false;
#if BLUETOOTH_ENABLED
    bluetoothInput.length = 0;
    bluetoothInput.droppingLine = false;
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
                      ConsoleJsonHandler sourceJsonHandler,
                      bool bluetoothSource = false) {
    while (input.available()) {
        char c = (char)input.read();

        if (bluetoothSource) {
            bluetoothRxBytes++;
            if (c == '\n') bluetoothRxLines++;
        }

        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (state.droppingLine) {
                state.length = 0;
                state.droppingLine = false;
                continue;
            }
            state.buffer[state.length] = '\0';
            dispatchLine(state.buffer, output, sourceJsonHandler);
            state.length = 0;
            continue;
        }

        if (!state.droppingLine && state.length < CONSOLE_LINE_BUFFER_SIZE - 1) {
            state.buffer[state.length++] = c;
        } else if (!state.droppingLine) {
            state.length = 0;
            state.droppingLine = true;
        }
    }
}

void console_update() {
    pollInput(Serial, Serial, usbInput, jsonHandler);
#if BLUETOOTH_ENABLED
    pollInput(BLUETOOTH_PORT, BLUETOOTH_PORT, bluetoothInput,
              bluetoothJsonHandler, true);
#endif
}

uint32_t console_bluetooth_rx_bytes() {
    return bluetoothRxBytes;
}

uint32_t console_bluetooth_rx_lines() {
    return bluetoothRxLines;
}
