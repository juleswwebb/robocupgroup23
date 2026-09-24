"""Keyboard trim and latched-drum safety checks without a Qt window."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from PyQt6.QtCore import Qt

from DebugGUI import RobotDebugGUI


class Value:
    def __init__(self, value):
        self._value = value

    def value(self):
        return self._value


class DriveControlTests(unittest.TestCase):
    def test_side_scales_apply_to_forward_and_reverse_not_pure_turns(self):
        gui = SimpleNamespace(
            drive_speed_slider=Value(100),
            drive_left_scale=Value(100),
            drive_right_scale=Value(98),
            drive_keys={Qt.Key.Key_W},
        )
        self.assertEqual(RobotDebugGUI._keyboard_drive_values(gui), (100, 98))
        gui.drive_speed_slider = Value(60)
        self.assertEqual(RobotDebugGUI._keyboard_drive_values(gui), (60, 59))
        gui.drive_keys = {Qt.Key.Key_S}
        self.assertEqual(RobotDebugGUI._keyboard_drive_values(gui), (-60, -59))
        gui.drive_keys = {Qt.Key.Key_A}
        self.assertEqual(RobotDebugGUI._keyboard_drive_values(gui), (-60, 60))

    def test_latched_drum_refreshes_and_stops_when_telemetry_stale(self):
        sent = []
        stopped = []
        gui = SimpleNamespace(
            drum_held=False,
            drum_latched=True,
            _drum_controls_available=lambda: True,
            last_telemetry_monotonic=time.monotonic(),
            drum_left_spin=Value(15),
            drum_right_spin=Value(20),
            bluetooth=SimpleNamespace(send_command=lambda *a, **k: sent.append((a, k))),
            drum_status_label=SimpleNamespace(setText=lambda _text: None),
            _stop_drum_hold=lambda: stopped.append(True),
        )
        with patch("DebugGUI.theme.set_pill_state"):
            RobotDebugGUI._send_drum_command(gui)
            self.assertEqual(sent, [(("drum_set",), {"left": 15, "right": 20})])
            gui.last_telemetry_monotonic -= 3
            RobotDebugGUI._send_drum_command(gui)
        self.assertEqual(stopped, [True])
        self.assertEqual(len(sent), 1)


if __name__ == "__main__":
    unittest.main()
