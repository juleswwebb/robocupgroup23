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

    def setValue(self, value):
        self._value = value


class DriveControlTests(unittest.TestCase):
    def test_explore_start_is_direct_and_does_not_require_a_mission(self):
        sent = []
        actions = []
        gui = SimpleNamespace(
            _exploration_controls_available=lambda: True,
            _stop_mission_route=lambda reason: actions.append(("mission", reason)),
            _set_drive_armed=lambda armed: actions.append(("keyboard", armed)),
            _stop_drum_hold=lambda: actions.append(("drum", "neutral")),
            explore_requested=False,
            explore_seen_active=False,
            explore_start_sent_at=None,
            bluetooth=SimpleNamespace(
                send_command=lambda *args, **kwargs: sent.append((args, kwargs))
            ),
            recorder=SimpleNamespace(record_command=lambda *_args: None),
            add_log=lambda *_args: None,
            _update_exploration_controls=lambda: None,
        )
        RobotDebugGUI._start_exploration(gui)
        self.assertEqual(sent, [(("explore_start",), {})])
        self.assertTrue(gui.explore_requested)
        self.assertEqual([item[0] for item in actions], ["mission", "keyboard", "drum"])

    def test_explore_keepalive_is_refreshed_over_the_connected_link(self):
        sent = []
        gui = SimpleNamespace(
            explore_requested=True,
            telemetry={"explore.active": True},
            bluetooth=SimpleNamespace(
                is_connected=lambda: True,
                send_command=lambda *args, **kwargs: sent.append((args, kwargs)),
            ),
            robot_debug_mode=True,
            explore_command_available=True,
            last_telemetry_monotonic=time.monotonic(),
        )
        RobotDebugGUI._send_explore_keepalive(gui)
        self.assertEqual(sent, [(("explore_keepalive",), {})])

    def test_servo_pulse_test_stops_to_neutral_on_release(self):
        sent = []
        gui = SimpleNamespace(
            servo_held=False,
            servo_test_pulse_us=1500,
            last_telemetry_monotonic=time.monotonic(),
            _servo_controls_available=lambda: True,
            _send_servo_test_pulse=lambda pulse: sent.append(pulse),
            _send_servo_test_command=lambda: RobotDebugGUI._send_servo_test_command(gui),
            servo_status_label=SimpleNamespace(setText=lambda _text: None),
        )
        with patch("DebugGUI.theme.set_pill_state"):
            RobotDebugGUI._start_servo_test(gui, 2000)
            self.assertEqual(sent, [2000])
            self.assertTrue(gui.servo_held)
            RobotDebugGUI._stop_servo_test(gui)
        self.assertEqual(sent, [2000, 1500])
        self.assertFalse(gui.servo_held)

    def test_servo_angle_sends_positional_target_on_fixed_d20(self):
        sent = []
        recorded = []
        gui = SimpleNamespace(
            servo_held=False,
            last_telemetry_monotonic=time.monotonic(),
            _servo_angle_controls_available=lambda: True,
            servo_angle_spin=Value(135),
            recorder=SimpleNamespace(
                record_command=lambda name, args: recorded.append((name, args))
            ),
            bluetooth=SimpleNamespace(
                send_command=lambda *args, **kwargs: sent.append((args, kwargs))
            ),
            servo_status_label=SimpleNamespace(setText=lambda _text: None),
        )
        with patch("DebugGUI.theme.set_pill_state"):
            RobotDebugGUI._send_servo_angle(gui)
        self.assertEqual(recorded, [("servo_angle_set", {"angle": 135})])
        self.assertEqual(sent, [(('servo_angle_set',), {"angle": 135})])

    def test_servo_center_uses_operator_calibrated_125_degrees(self):
        sent_angles = []
        gui = SimpleNamespace(
            servo_angle_spin=Value(90),
            servo_angle_slider=Value(90),
            _send_servo_angle=lambda: sent_angles.append(gui.servo_angle_spin.value()),
        )
        RobotDebugGUI._center_servo(gui)
        self.assertEqual(gui.servo_angle_spin.value(), 125)
        self.assertEqual(gui.servo_angle_slider.value(), 125)
        self.assertEqual(sent_angles, [125])

    def test_servo_test_refuses_to_start_without_fresh_telemetry(self):
        sent = []
        gui = SimpleNamespace(
            servo_held=False,
            servo_test_pulse_us=1500,
            last_telemetry_monotonic=time.monotonic() - 3.0,
            _servo_controls_available=lambda: True,
            _send_servo_test_pulse=lambda pulse: sent.append(pulse),
            servo_status_label=SimpleNamespace(setText=lambda _text: None),
        )
        with patch("DebugGUI.theme.set_pill_state"):
            RobotDebugGUI._start_servo_test(gui, 1000)
        self.assertEqual(sent, [])
        self.assertFalse(gui.servo_held)

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

    def test_keyboard_drive_does_not_cancel_continuous_drum(self):
        sent = []
        drum_stopped = []
        gui = SimpleNamespace(
            _drive_controls_available=lambda: True,
            drive_arm_checkbox=SimpleNamespace(isChecked=lambda: True),
            _keyboard_drive_values=lambda: (85, 100),
            route_follower=None,
            drum_held=False,
            drum_latched=True,
            _stop_drum_hold=lambda: drum_stopped.append(True),
            bluetooth=SimpleNamespace(send_command=lambda *a, **k: sent.append((a, k))),
            drive_status_label=SimpleNamespace(setText=lambda _text: None),
        )
        with patch("DebugGUI.theme.set_pill_state"):
            RobotDebugGUI._send_keyboard_drive(gui)
        self.assertEqual(sent, [(("drive_set",), {"left": 85, "right": 100})])
        self.assertEqual(drum_stopped, [])

    def test_magnet_keepalive_requires_fresh_telemetry(self):
        sent = []
        off_requests = []
        gui = SimpleNamespace(
            magnet_is_on=True,
            _magnet_controls_available=lambda: True,
            last_telemetry_monotonic=time.monotonic(),
            magnet_status_label=object(),
            bluetooth=SimpleNamespace(
                send_command=lambda *args, **kwargs: sent.append((args, kwargs))
            ),
            _request_magnet=lambda enabled, status: off_requests.append((enabled, status)),
        )

        with patch("DebugGUI.theme.set_pill_state"):
            RobotDebugGUI._send_magnet_keepalive(gui)
            self.assertEqual(sent, [(("magnet_set",), {"enabled": True})])

            gui.last_telemetry_monotonic -= 3.0
            RobotDebugGUI._send_magnet_keepalive(gui)
        self.assertEqual(len(sent), 1)
        self.assertEqual(off_requests, [(False, "OFF · TELEMETRY LOST")])


if __name__ == "__main__":
    unittest.main()
