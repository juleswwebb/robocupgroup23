# RobocupGroup23

ENMT301 Robocup firmware, targeting the Teensy 4.0 controller.

## Getting started

1. Install [VS Code](https://code.visualstudio.com/) and the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode).
2. Open this folder in VS Code (`File > Open Folder...`).
3. PlatformIO will pick up `platformio.ini` automatically. Use the PlatformIO toolbar (bottom status bar) to Build, Upload, and open the Serial Monitor.

Or from the command line:

```
pio run            # build
pio run -t upload  # build and flash
pio device monitor # serial monitor
```

## Project structure

- `src/main.cpp` — task scheduler setup and the periodic tasks that drive the robot.
- `src/*.cpp` + `include/*.h` — one module per subsystem (`motors`, `sensors`, `weight_collection`, `return_to_base`). Each function is currently a stub — fill these in as sensors/actuators get wired up.
- `platformio.ini` — board config (`teensy40`) and library dependencies. Extra libraries for TOF sensors, IMU, smart servo etc. are listed commented-out; uncomment as you add that hardware.

See the previous year's group code (`../robocup code PrevGroup`) for a working reference implementation of PID drive, TOF/ultrasonic navigation, IMU watchdogs, and the servo gripper.
