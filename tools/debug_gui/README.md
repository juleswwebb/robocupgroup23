# Robot Debug Console

Live telemetry, plotting, tuning and session recording for the robot. The app
talks the same newline-delimited JSON over direct Teensy USB Serial or the
matched CH9143 Bluetooth serial bridge. Full
protocol spec:
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

In `DebugGUI.py`: pick either the Teensy USB port or the CH9143 receiver's
virtual serial port (`/dev/cu.usbmodem*` on macOS, `COMx` on Windows), leave
baud at 115200, and hit **Connect**. Firmware upload still requires USB. The firmware
detects the GUI's `hello` message, silences text prints and starts JSON
telemetry on that same port.

## What the firmware exposes

Telemetry signals (sent at 10 Hz by default in one grouped packet):

| Prefix | Signals |
|---|---|
| `tof.*` | `xshut0`–`xshut6`, `8x8`, `serial`, `array_min`, `array_valid_zones`, and `array.r0c0`–`array.r7c7` (mm) |
| `ir.*` | `0`–`3` (mm) |
| `ultrasonic.*` | `0`, `1` (mm) |
| `colour.*` | `r`, `g`, `b`, `c` |
| `imu.*` | `heading`, `roll`, `pitch`, `cal_system`, `cal_gyro`, `cal_accel`, `cal_mag` |
| `flow.*` | `dx`, `dy`, `total_x`, `total_y` |
| `inductive.*` | `detected`, `count` |
| `encoder.*` | `0`, `1` |
| `servo.*` | `us` |
| `drive.*` | `left_percent`, `right_percent`, `left_us`, `right_us`, `active` |
| `bluetooth.*` | `active`, `rx_messages` |
| `system.*` | `uptime_ms` |

A sensor that failed to initialise (or has nothing in range) is sent as JSON
`null`. The GUI shows this as `INVALID`, so an old value is never mistaken for
a current reading. Firmware-provided telemetry definitions supply the friendly
name, group and unit shown in the live table.

The dedicated **8×8 TOF** tab displays all 64 zones as a live colour map. Any
zone can be selected and sent straight to the normal plotter. Every other
numeric live value is also automatically available in the **Plots** tab.

Commands (Commands tab / Dashboard):

- `stop` — every actuator to neutral. Always allowed.
- `set_debug_mode` — gate the actuator commands below.
- `servo_set` — `us` / `speed` / `angle`. Debug mode only.
- `drive_set` — independent left/right main-drive percentages. Debug mode only.
- `encoders_reset` — zero both encoder counts.
- `set_text_mode` — drop back to human-readable serial output.

Parameters (Parameters tab, live-tunable):

- `telemetry.interval_ms` — how often telemetry packets are sent.
- `servo.pulse_us` — servo pulse width.
- `drive.max_percent` — main-drive test speed limit (hard-capped at 100%).

The Dashboard has a dedicated **Keyboard Drive** panel. Enable Debug Mode,
arm the panel, click the app window, then use **W/S** or **↑/↓** for
forward/reverse and **A/D** or **←/→** to turn. Space or releasing every drive
key sends neutral. The Teensy
also makes both drive outputs neutral after 300 ms without a new command.

Adding a new signal only needs a firmware change (add it to the telemetry
packet in `src/DebugProtocol.cpp`); the GUI discovers it automatically.

## Transport test suite

`RobotProtocolTest.py` performs a non-moving end-to-end check of one or more
ports. It verifies handshake, every expected sensor/ToF key, definitions,
parameter round-trips, STOP, Debug Mode, and zero-speed drive/servo commands.

```bash
python RobotProtocolTest.py --list
python RobotProtocolTest.py /dev/cu.usbmodem145902401 \
  /dev/cu.usbmodemWCH285EB3TS11
```

If USB passes but the CH9143 port receives zero messages, connect over USB and
inspect `bluetooth.rx_messages`. Zero after a Bluetooth probe means the Teensy
never received the app's `hello`: check radio power/pairing and crossed wiring
(CH9143 TX -> Teensy RX1 D0, CH9143 RX -> Teensy TX1 D1). Firmware upload
always remains USB-only.

## Appearance

`theme.py` holds the shared dark theme - palette, Qt stylesheet, pyqtgraph
styling, and the window sizing helper. Both GUIs import it, so a colour or
spacing change made there lands in both. Windows size themselves to the screen
they open on rather than assuming a large display.

## Colour sensor card

The Dashboard shows the colour sensor as an actual colour swatch, with a
best-guess colour name, hex code, raw RGBC counts and a strip of recent
colours. Uncalibrated it shows hue only; hold the sensor over something white
at its working distance and click **Calibrate White** so brightness means
something too (black reads as black). The calibration is remembered.

## 8×8 depth camera

The **8×8 TOF** tab turns the SEN0628 (60° field of view, 3.5 m range) into a
depth camera:

- **Depth image** — near is red, far is blue. Click a zone for its distance
  and angle; double-click to plot it.
- **Top-down map** — every zone projected into millimetres in front of the
  sensor.
- **Objects** — clusters of nearby zones with bearing, distance and
  approximate width. For real use, point the sensor at the empty arena and
  click **Capture Background** (averages 10 frames): after that only things
  nearer than the background count, so the floor and walls don't.
- Frame smoothing, mirror/flip (wave a hand on one side to check), freeze,
  and PNG/CSV export. All settings persist between runs.

## Wiring map

The **Wiring** tab records what is plugged in where. Give each sensor or
actuator a human name ("Front top right ToF"), pick its type, and pick its
port — each type only offers ports that suit it (XSHUT lines for VL53s,
Serial1–8 for the serial ToF, A0–A13 for IR and so on). The name then
replaces the raw key (`tof.xshut3`) across the dashboard and plots.

- Saves automatically to `hardware_map.json` beside the app. It is committed,
  so the whole group shares one map.
- The firmware reads every port it supports, so after re-plugging a sensor
  just change its port here — the name follows the data, no reflash.
- The Status column flags a port or Teensy pin used twice, a port the
  firmware doesn't read, and a sensor type the firmware doesn't expect on
  that port (e.g. a VL53L0X on an XSHUT line driven as a VL53L1X).
- Every recording stores a snapshot of the map in its metadata.

The port catalogue in `wiring.py` mirrors `include/sensor_config.h`; if a
pin assignment changes there, update the catalogue to match.

## Recordings

Sessions are saved as `.rdbg` files (SQLite) in `Data/`, which is gitignored —
recordings are test artefacts, not source. `DataAnalysisCLI.py` has `summary`,
`faults`, `stats`, `correlate`, `pid`, `anomalies` and `report` subcommands for
working with them from the terminal.
