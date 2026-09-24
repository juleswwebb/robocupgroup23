"""The Bluetooth handshake retry must be infrequent and actuator-safe."""

import json
import unittest
from unittest.mock import patch

from BluetoothSerial import SerialWorker


class BluetoothRecoveryTests(unittest.TestCase):
    def test_retries_only_after_stale_telemetry(self):
        worker = SerialWorker("unused")
        worker._last_telemetry_at = 0.0
        worker._last_recovery_hello_at = 0.0

        with patch("BluetoothSerial.time.monotonic", return_value=11.9):
            worker._recover_stale_telemetry()
        self.assertTrue(worker._tx_queue.empty())

        with patch("BluetoothSerial.time.monotonic", return_value=12.0):
            worker._recover_stale_telemetry()
        self.assertEqual(json.loads(worker._tx_queue.get_nowait())["type"], "hello")

        with patch("BluetoothSerial.time.monotonic", return_value=20.0):
            worker._recover_stale_telemetry()
        self.assertTrue(worker._tx_queue.empty())

        worker._last_telemetry_at = 23.0
        with patch("BluetoothSerial.time.monotonic", return_value=25.0):
            worker._recover_stale_telemetry()
        self.assertTrue(worker._tx_queue.empty())


if __name__ == "__main__":
    unittest.main()
