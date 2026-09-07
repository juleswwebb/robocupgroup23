#include "Console.h"
#include <Arduino.h>
#include <string.h>
#include <ctype.h>

// Sized for the debug console's JSON messages, which are far longer than
// the hand-typed text commands (a command with several arguments can run
// well past a hundred characters).
#define CONSOLE_LINE_BUFFER_SIZE 256

static char lineBuffer[CONSOLE_LINE_BUFFER_SIZE];
static uint8_t lineLength = 0;
static ConsoleCommandHandler commandHandler = nullptr;
static ConsoleJsonHandler jsonHandler = nullptr;

void console_set_command_handler(ConsoleCommandHandler handler) {
    commandHandler = handler;
}

void console_set_json_handler(ConsoleJsonHandler handler) {
    jsonHandler = handler;
}

void console_init() {
    lineLength = 0;
}

static void dispatchLine(char* line) {
    // Split off the first whitespace-separated token as the command;
    // everything after it (trimmed) is passed through as args.
    char* command = line;
    while (*command == ' ') command++;

    // JSON goes to the debug protocol untouched - the lowercasing below
    // would otherwise corrupt string values inside the message.
    if (*command == '{') {
        if (jsonHandler) {
            jsonHandler(command);
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
        commandHandler(command, args);
    }
}

void console_update() {
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            lineBuffer[lineLength] = '\0';
            dispatchLine(lineBuffer);
            lineLength = 0;
            continue;
        }

        if (lineLength < CONSOLE_LINE_BUFFER_SIZE - 1) {
            lineBuffer[lineLength++] = c;
        }
        // else: silently drop overflow chars, line will still dispatch on '\n'
    }
}
