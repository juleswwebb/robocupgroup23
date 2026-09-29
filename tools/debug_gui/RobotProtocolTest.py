#!/usr/bin/env python3
"""End-to-end test for the Group 23 USB/Bluetooth robot protocol.

The test is actuator-safe: it only sends STOP and zero-speed commands. It can
test one or more serial ports sequentially, proving that the same GUI protocol
works over direct Teensy USB and the CH9143 virtual serial port.

Examples:
    python RobotProtocolTest.py --list
    python RobotProtocolTest.py /dev/cu.usbmodem145902401
    python RobotProtocolTest.py /dev/cu.usbmodem145902401 \
        /dev/cu.usbmodemWCH285EB3TS11
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from typing import Any

import serial
from serial.tools import list_ports


BAUD = 115200
REQUIRED_COMMANDS = {
    "stop", "set_debug_mode", "drive_set", "drum_set", "magnet_set", "encoders_reset",
}
REQUIRED_PARAMETERS = {
    "telemetry.interval_ms", "drive.max_percent",
}
REQUIRED_TELEMETRY = {
    *(f"tof.xshut{i}" for i in range(7)),
    "tof.8x8", "tof.array_min", "tof.array_valid_zones",
    *(f"ir.{i}" for i in range(4)),
    "ultrasonic.0", "ultrasonic.1",
    "colour.r", "colour.g", "colour.b", "colour.c",
    "imu.heading", "imu.roll", "imu.pitch",
    "flow.dx", "flow.dy", "flow.total_x", "flow.total_y",
    "inductive.detected", "inductive.count",
    "encoder.0", "encoder.1",
    "drive.left_percent", "drive.right_percent",
    "drive.left_us", "drive.right_us", "drive.active",
    "drum.left_percent", "drum.right_percent",
    "drum.left_us", "drum.right_us", "drum.active",
    "magnet.on",
    "bluetooth.active", "bluetooth.rx_messages",
    "system.uptime_ms",
}


@dataclass
class ProbeResult:
    port: str
    messages: int = 0
    telemetry_packets: int = 0
    malformed_json: int = 0
    raw_text: list[str] = field(default_factory=list)
    telemetry_definitions: set[str] = field(default_factory=set)
    command_definitions: set[str] = field(default_factory=set)
    parameter_definitions: set[str] = field(default_factory=set)
    parameter_values: dict[str, Any] = field(default_factory=dict)
    telemetry_keys: set[str] = field(default_factory=set)
    states: list[dict] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    logs: list[str] = field(default_factory=list)


def available_ports() -> list[dict[str, str]]:
    return [
        {
            "device": item.device,
            "description": item.description or "",
            "manufacturer": item.manufacturer or "",
            "hwid": item.hwid or "",
        }
        for item in sorted(list_ports.comports(), key=lambda value: value.device)
    ]


def send_message(link: serial.Serial, payload: dict[str, Any]) -> None:
    encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
    for offset in range(0, len(encoded), 20):
        link.write(encoded[offset:offset + 20])
        if offset + 20 < len(encoded):
            time.sleep(0.003)
    link.flush()


def receive_for(link: serial.Serial, result: ProbeResult, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        raw = link.readline()
        if not raw:
            continue
        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            continue
        try:
            message = json.loads(text)
        except json.JSONDecodeError:
            result.raw_text.append(text)
            if text.startswith("{"):
                result.malformed_json += 1
            continue

        if not isinstance(message, dict):
            continue
        result.messages += 1
        kind = message.get("type")

        if kind == "telemetry_definition":
            result.telemetry_definitions.add(str(message.get("name", "")))
        elif kind == "command_definition":
            result.command_definitions.add(str(message.get("name", "")))
        elif kind == "parameter_definition":
            result.parameter_definitions.add(str(message.get("name", "")))
        elif kind == "parameter_value":
            result.parameter_values[str(message.get("name", ""))] = message.get("value")
        elif kind == "telemetry":
            result.telemetry_packets += 1
            data = message.get("data", {})
            if isinstance(data, dict):
                result.telemetry_keys.update(str(key) for key in data)
        elif kind == "state":
            result.states.append(message)
        elif kind == "error":
            result.errors.append(str(message.get("message", "unknown error")))
        elif kind == "log":
            result.logs.append(str(message.get("message", "")))


def probe(port: str, baud: int, timeout: float) -> ProbeResult:
    result = ProbeResult(port=port)
    with serial.Serial(port, baudrate=baud, timeout=0.10, write_timeout=1.0) as link:
        time.sleep(0.25)
        link.reset_input_buffer()

        send_message(link, {"type": "hello", "client": "RobotProtocolTest", "protocol": 1})
        send_message(link, {"type": "request_definitions"})
        receive_for(link, result, timeout)

        # Exercise command and parameter paths without moving any actuator.
        send_message(link, {"type": "command", "command": "stop"})
        send_message(link, {"type": "command", "command": "set_debug_mode", "enabled": True})
        send_message(link, {"type": "command", "command": "drive_set", "left": 0, "right": 0})
        send_message(link, {"type": "command", "command": "drum_set", "left": 0, "right": 0})
        send_message(link, {"type": "command", "command": "magnet_set", "enabled": False})
        send_message(link, {"type": "parameter_request", "name": "drive.max_percent"})
        send_message(link, {"type": "parameter_request", "name": "telemetry.interval_ms"})
        send_message(link, {"type": "command", "command": "set_debug_mode", "enabled": False})
        receive_for(link, result, 2.0)

    return result


def validate(result: ProbeResult) -> list[str]:
    failures: list[str] = []
    missing_commands = REQUIRED_COMMANDS - result.command_definitions
    missing_parameters = REQUIRED_PARAMETERS - result.parameter_definitions
    missing_definitions = REQUIRED_TELEMETRY - result.telemetry_definitions
    missing_values = REQUIRED_TELEMETRY - result.telemetry_keys

    if result.messages == 0:
        failures.append("no JSON messages received")
    if result.telemetry_packets == 0:
        failures.append("no telemetry packets received")
    if missing_commands:
        failures.append(f"missing commands: {', '.join(sorted(missing_commands))}")
    if missing_parameters:
        failures.append(f"missing parameters: {', '.join(sorted(missing_parameters))}")
    if missing_definitions:
        failures.append(f"missing telemetry definitions: {', '.join(sorted(missing_definitions))}")
    if missing_values:
        failures.append(f"missing telemetry values: {', '.join(sorted(missing_values))}")
    if result.malformed_json:
        failures.append(f"{result.malformed_json} malformed JSON line(s)")
    if result.errors:
        failures.append(f"firmware errors: {'; '.join(result.errors)}")
    if not any(state.get("debug_mode") is False for state in result.states):
        failures.append("did not confirm final safe/non-debug state")
    if "drive.max_percent" not in result.parameter_values:
        failures.append("drive.max_percent did not round-trip")
    if "telemetry.interval_ms" not in result.parameter_values:
        failures.append("telemetry.interval_ms did not round-trip")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ports", nargs="*", help="USB and/or CH9143 serial device paths")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--timeout", type=float, default=8.0,
                        help="seconds to wait for definitions and telemetry per port")
    parser.add_argument("--list", action="store_true", help="list ports and exit")
    args = parser.parse_args()

    if args.list:
        for item in available_ports():
            print(f"{item['device']}: {item['description']} | {item['manufacturer']} | {item['hwid']}")
        return 0

    if not args.ports:
        parser.error("provide at least one serial port, or use --list")

    overall_ok = True
    for port in args.ports:
        print(f"\nTesting {port} at {args.baud} baud...")
        try:
            result = probe(port, args.baud, args.timeout)
            failures = validate(result)
        except (OSError, serial.SerialException) as exc:
            print(f"FAIL: could not test port: {exc}")
            overall_ok = False
            continue

        print(
            f"Received {result.messages} JSON messages, "
            f"{result.telemetry_packets} telemetry packets, "
            f"{len(result.telemetry_definitions)} signal definitions, "
            f"{len(result.command_definitions)} commands."
        )
        if failures:
            overall_ok = False
            for failure in failures:
                print(f"FAIL: {failure}")
        else:
            print("PASS: handshake, telemetry, definitions and safe commands all worked.")

    return 0 if overall_ok else 1


if __name__ == "__main__":
    sys.exit(main())
