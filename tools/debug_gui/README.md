# Robot Debug Console

Live telemetry, plotting, tuning and session recording for the robot, talking
newline-delimited JSON over a serial link (USB now, CH9143 Bluetooth later -
the protocol doesn't care which). Full protocol spec:
[`docs/communicationProtocol.md`](../../docs/communicationProtocol.md).

## Setup

```bash
cd tools/debug_gui
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Running

```bash
python DebugGUI.py          # live console: telemetry, plots, tuning, recording
python DataVisualiser.py    # offline viewer for recorded .rdbg files
python DataAnalysisCLI.py summary Data/SomeRun.rdbg   # headless analysis
```

**Important:** close the PlatformIO serial monitor before connecting the GUI.
Only one program can hold the serial port at a time.

In `DebugGUI.py`: pick the Teensy's port (`/dev/cu.usbmodem*` on macOS, `COMx`
on Windows), leave baud at 115200, and hit **Connect**. The firmware detects
the GUI's `hello` message and automatically switches from human-readable text
output to JSON telemetry, so the sensor values populate on their own.

## What the firmware exposes

Telemetry signals (all sent ~20x/second in one grouped packet):

| Prefix | Signals |
|---|---|
| `tof.*` | `xshut0`–`xshut6` (mm), `array_centre`, `array_min`, `serial` |
| `ir.*` | `0`–`3` (mm) |
| `ultrasonic.*` | `0`, `1` (mm) |
| `colour.*` | `r`, `g`, `b`, `c` |
| `imu.*` | `heading`, `roll`, `pitch`, `cal_system`, `cal_gyro`, `cal_accel`, `cal_mag` |
| `flow.*` | `dx`, `dy`, `total_x`, `total_y` |
| `inductive.*` | `detected`, `count` |
| `encoder.*` | `0`, `1` |
| `servo.*` | `us` |
| `system.*` | `uptime_ms` |

A sensor that failed to initialise (or has nothing in range) simply doesn't
appear in the packet, rather than reporting a misleading zero.

Commands (Commands tab / Dashboard):

- `stop` — servo to neutral. Always allowed.
- `set_debug_mode` — gate the actuator commands below.
- `servo_set` — `us` / `speed` / `angle`. Debug mode only.
- `encoders_reset` — zero both encoder counts.
- `set_text_mode` — drop back to human-readable serial output.

Parameters (Parameters tab, live-tunable):

- `telemetry.interval_ms` — how often telemetry packets are sent.
- `servo.pulse_us` — servo pulse width.

Adding a new signal only needs a firmware change (add it to the telemetry
packet in `src/DebugProtocol.cpp`); the GUI discovers it automatically.

## Appearance

`theme.py` holds the shared dark theme - palette, Qt stylesheet, pyqtgraph
styling, and the window sizing helper. Both GUIs import it, so a colour or
spacing change made there lands in both. Windows size themselves to the screen
they open on rather than assuming a large display.

## Recordings

Sessions are saved as `.rdbg` files (SQLite) in `Data/`, which is gitignored —
recordings are test artefacts, not source. `DataAnalysisCLI.py` has `summary`,
`faults`, `stats`, `correlate`, `pid`, `anomalies` and `report` subcommands for
working with them from the terminal.
