# Isolated CH9143 probe firmware

This small PlatformIO project tests the Teensy 4.0 `Serial1`/CH9143 path without
starting any robot sensors, schedulers or actuators. It uses the same 115200
baud rate and extra RX/TX buffers as the main application.

It sends a `probe_beacon` JSON line once per second and echoes every received
byte. The Teensy USB serial connection reports the cumulative received-byte
count. Uploading it temporarily replaces the robot firmware, so upload the main
project again after testing.

```sh
platformio run --project-dir tools/bluetooth_probe_firmware --target upload
```
