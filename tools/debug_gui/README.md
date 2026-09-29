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
| `tof.*` | `xshut3`–`xshut8`, `8x8`, `serial`, `array_min`, `array_valid_zones`, and `array.r0c0`–`array.r7c7` (mm) |
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

The dedicated **8×8 TOF** tab opens as a numbered raw 8×8 distance grid
(millimetres). Missing returns appear as dashes, not as an invented distance.
Use **View** to switch to a smoothed image or Group 7-inspired stable-detail
filtering. These filters affect the visualisation only; the raw sensor values,
CSV export and object detection are not rewritten. The sidebar offers an
automatic colour range and orientation controls. Click a zone to inspect it,
or double-click to send it to **Plots**. Every other numeric live value is
also automatically available in the **Plots** tab.

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
- the **Use 2 measured runs** button applies the 1805/1830 mm test averages:
  encoder 0 = 0.08833 mm/count, encoder 1 = 0.08609 mm/count with encoder 1
  inverted for forward travel. These are also the defaults on a fresh install;
  untouched legacy 0.095/0.095 settings are upgraded once, while custom saved
  calibrations are preserved. These are provisional odometry scales, not a drive
  motor correction; a drifting powered run cannot distinguish motor speed
  imbalance from wheel-size or encoder-count differences;
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

The Dashboard also has a new **Autonomous Explore** control for arena-free
testing. It starts robot-side exploration directly; no placed weights or map
upload is involved. The Teensy drives at the configured straight trim (default
100% left / 85% right), keeps the D28/D29 drum motors at -100% / -100% and
the D26 electromagnet on, and
uses multi-pixel 8×8 TOF evidence to turn around a detected front obstacle.
The inward-angled top/bottom VL53 pairs are used for three-sample weight
detection, steady centering turns, and a forward pass through the target. A
lower return must be within 500 mm and at least 120 mm nearer than its recent
background, while its upper partner does not see it close. The
run requires Debug Mode, a drive limit of at least 80%, and valid 8×8 or
straight-ahead range data; IMU/encoder calibration and Mission Planner setup
are not prerequisites. STOP, Debug Mode exit, manual motor takeover, or a
700 ms lost app keepalive neutralises the drive and drum and switches off the
magnet. Avoidance turns at a coherent 8×8 return around 420 mm; the robot keeps
turning until the front is clear for two fresh frames. If it cannot clear the
front after five seconds, it stops instead of driving into the obstruction.
Hardware behavior is untested, so begin supervised in a clear area.

The separate **Mission Planner** tab follows Group 7's newer pre-laid arena
workflow: a 4.9 m × 2.4 m arena with green/blue homes, an opposite-home no-go
zone, draggable start/real weights/dummy weights/walls/ramps/tubes, adjustable
obstacle dimensions and clearance, and a route through real weights with an
optional return-home leg. Hollow pink circles are unconfirmed live TOF weight
candidates transformed from the Arena View pose. The plan is saved in desktop
settings. New walls default to 600 × 130 mm. Select a wall or ramp to enter
any angle in the Rotation field; the drawing and route clearance use the
rotated shape. Existing saved obstacle sizes and angles are retained.

After planning, **FOLLOW ROUTE** can command the existing drive motors from
the desktop app over USB or Bluetooth. It requires Debug Mode,
fresh encoder/gyro telemetry, a healthy 8×8 ToF frame, a stopped robot,
and explicit confirmation that the physical start pose matches the marked
start and arrow. The app must remain open and connected; it sends fresh
commands every 100 ms while the Teensy's independent 300 ms drive watchdog
remains active. Each moving wheel is commanded at 80–100% because this
drivetrain stalls below 80%; route start requests and waits for confirmation
of a 100% firmware drive limit. Straight travel commands default to 100% left
and 85% right; small heading corrections stay within the 80–100% moving
range. A healthy all-out-of-range 8×8 frame is clear space and permits
movement; a failed/missing frame-health signal stops the route, so upload the
matching firmware. Route obstacle mapping reads all six installed VL53L1X
point sensors on the SX1509 at 0x71: IO8/5/6/7 are the inward-facing front
top/bottom pairs, while IO9/10 are the straight-ahead top right/left wall
sensors. Their positions, angles and enabled state come from Arena View,
keyed by XSHUT channel, so Wiring-tab renames do not move them. The central
upper 8×8 field also supplies candidate points. A candidate is mapped only
after it reappears near the same world point across three telemetry frames,
so a single transient sensor return does not trigger a detour. The 8×8 rows
0–3 in central columns are eligible; row 4
and lower are ignored because this mounting sees persistent floor/chassis
returns there. Invalid/saturated ToF values are ignored. Side ultrasonics are
configured sideways and shown as wall-range rays in Arena View; they are not
inserted as generic obstacle points because that would make arena walls look
like objects to drive around.

Mapped obstacles appear as red rings and are transient. If one blocks the
remaining route, the app commands STOP, replans from the current encoder/IMU
pose, then resumes only with a clear detour. If that pose overlaps only the
extra planning buffer around a live sensor return or arena edge, a clear first
leg moving away is allowed; otherwise the detour starts with a short waypoint
out of that buffer. An actual robot-footprint overlap remains a hard failure.
The current operator-set obstacle mapping and immediate-stop threshold is
strictly below 50 mm of raw sensor range. A 430 mm return is therefore visible in Arena View but does not create
a Mission Planner detour. This is a testing setting, not a verified stopping
distance for a robot moving at 80–100%.
No safe detour, repeated replans, missing/stale telemetry, pose jumps, excess
path deviation, no progress, operator takeover, and STOP still halt the route.
Replan-stop records include a mission-map snapshot and the exact blocked-pose
reason (arena edge, drawn obstacle, exclusion zone, or live sensor return) for
diagnosis. The follower does **not** collect weights, unload, or upload the
route to the Teensy. Arena View's separate route remains preview-only. Because
80–100% is a high output and obstacle projection depends on calibrated sensor
placement, begin hardware testing with the wheels raised and then in a clear,
supervised area with an accessible STOP button. These are provisional odometry
scales, not absolute arena localisation. Run
offline regressions with `QT_QPA_PLATFORM=offscreen .venv/bin/python -m unittest discover -s tools/debug_gui -p 'test_*.py'` from the repo root.

Parameters (Parameters tab, live-tunable):

- `telemetry.interval_ms` — how often telemetry packets are sent.
- `drive.max_percent` — main-drive test speed limit (hard-capped at 100%).

The Dashboard has a dedicated **Keyboard Drive** panel. Enable Debug Mode,
arm the panel, click the app window, then use **W/S** or **↑/↓** for
forward/reverse and **A/D** or **←/→** to turn. Space or releasing every drive
key sends neutral. **Left scale** and **Right scale** set each side as a
percentage of the speed-limit slider during forward/reverse travel; e.g.
100% left and 85% right gives 100/85 at full speed or 60/51 at 60% speed.
Pure turns remain symmetric. The scales are saved locally in the app and
never arm the motors on startup. The Teensy
also makes both drive outputs neutral after 300 ms without a new command.

The Dashboard's **Drum motors** panel tests two independent servo-style driver
channels on D28 (left) and D29 (right). Set signed percentages and either
press-and-hold RUN for a momentary test or toggle **Continuous RUN** to latch
the selected speeds. The app refreshes a latched command every 100 ms; it
stops on toggle-off, STOP, leaving Debug Mode, disconnection, stale telemetry,
or app exit. Unlike momentary hold, it keeps running if you switch to another
app while the robot link remains healthy. A lost app/link also trips the
firmware's independent 300 ms timeout. The firmware caps commands at ±100%
and starts neutral.
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
port — each type only offers ports that suit it (SX1509 channels for VL53s,
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
