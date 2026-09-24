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

The firmware currently streams telemetry to the **most recently active**
USB or Bluetooth GUI, not both simultaneously. If the Bluetooth port opens
but the Dashboard says **NO TELEMETRY**, close any other GUI session using
USB. The app retries its Bluetooth handshake after a quiet link (at most once
every 12 seconds); reconnecting the selected port also requests the stream
immediately. A green connected pill only confirms that the computer opened
the serial device; it does not by itself prove radio data is arriving.

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
| `drive.*` | `left_percent`, `right_percent`, `left_us`, `right_us`, `active` |
| `bluetooth.*` | `active`, `rx_messages` |
| `system.*` | `uptime_ms` |

The current robot has no installed IR or colour sensor. Their firmware keys
remain available for later hardware work, but the live GUI hides them from
telemetry and plots and no longer shows the colour card. Raw Serial remains
an unfiltered diagnostic view of what the firmware actually sends.

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
- `drive_set` — independent left/right main-drive percentages. Debug mode only.
- `encoders_reset` — zero both encoder counts.
- `set_text_mode` — return to human-readable serial output.
- `navigation_set` — start/stop the safety-gated autonomous navigator. Starting
  requires Debug Mode, a valid BNO055 reading and a valid forward 8×8 range.

## Arena View

The **Arena View** tab consumes the same live telemetry over USB or Bluetooth
and builds a local robot-centred map. It provides:

- encoder/BNO055 odometry, robot trail and heading;
- point-TOF and 8×8 range rays with accumulated obstacle endpoints;
- persistent x/y position, aim and height-above-ground controls for the
  installed VL53, 8×8 ToF and ultrasonic sensors. Their names come from the
  Wiring tab and update when a device is renamed. Gold markers are VL53,
  purple is 8×8 ToF, and cyan is ultrasonic. In **Place sensor** mode, choose
  a named sensor in the placement panel before dragging co-located markers;
- optical flow and inductive markers remain placeable; IMU and encoders still
  feed odometry but have no placement marker. Inactive colour/IR hardware is
  omitted. Height is layout metadata; this top-down map does not yet use it
  to project sloped rays or alter obstacle locations;
- measured-distance encoder calibration: with the drive motors OFF, capture
  start counts, roll straight forward a measured distance, then calculate
  each wheel's mm/count and count polarity. Verify with a second run;
- manual walls, obstacle circles, weights and a goal, with an A* **preview**
  route inflated by robot radius and clearance;
- three-frame top/bottom TOF depth-gap hints for possible weights;
- separate left/right encoder scale, direction and wheel-track calibration;
- pan/zoom controls and a resettable local origin;
- navigation start/stop and live tuning for speed, turn speed and forward
  obstacle distance.

This is local dead reckoning, not absolute arena localisation. Calibrate the
encoders and physically verify every sensor pose before using the map for
navigation. Autonomous navigation is disabled at boot and the global STOP,
manual drive, Debug Mode exit, invalid IMU, missing range data and turn timeout
all return the drive outputs to neutral.

The separate **Mission Planner** tab follows Group 7's newer pre-laid arena
workflow: a 4.9 m × 2.4 m arena with green/blue homes, an opposite-home no-go
zone, draggable start/real weights/dummy weights/walls/ramps/tubes, adjustable
obstacle dimensions and clearance, and a route through real weights with an
optional return-home leg. Hollow pink circles are unconfirmed live TOF weight
candidates transformed from the Arena View pose. The plan is saved in desktop
settings, but **it is not uploaded to the Teensy and cannot drive the robot**.
Calibrate encoder distance, pose origin and sensor directions before attempting
firmware waypoint following. Run offline planner regressions with
`python -m unittest test_arena_planner test_mission_layout` in this directory.

Parameters (Parameters tab, live-tunable):

- `telemetry.interval_ms` — how often telemetry packets are sent.
- `drive.max_percent` — main-drive test speed limit (hard-capped at 100%).

The Dashboard has a dedicated **Keyboard Drive** panel. Enable Debug Mode,
arm the panel, click the app window, then use **W/S** or **↑/↓** for
forward/reverse and **A/D** or **←/→** to turn. Space or releasing every drive
key sends neutral. The Teensy
also makes both drive outputs neutral after 300 ms without a new command.

The Dashboard's **Drum motors** panel tests two independent servo-style driver
channels on D28 (left) and D29 (right). Set signed percentages and
press-and-hold RUN; releasing the button sends neutral. The firmware caps
commands at ±100%, starts neutral, and times out after 300 ms without commands.
This assumes the same 1050/1500/1950 µs interface as Group 7's DFR0513;
confirm your actual driver and connector before plugging it in. D28/D29 are
signal pins, not motor power. STOP and leaving Debug Mode neutralise both.

Adding a new signal only needs a firmware change (add it to the telemetry
packet in `src/DebugProtocol.cpp`); the GUI discovers it automatically.

## Transport test suite

`RobotProtocolTest.py` performs a non-moving end-to-end check of one or more
ports. It verifies handshake, every expected sensor/ToF key, definitions,
parameter round-trips, STOP, Debug Mode, and zero-speed drive/drum commands.

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
