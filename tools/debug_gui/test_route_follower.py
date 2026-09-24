"""Offline route-following regressions; these tests never command hardware."""

import math
import time
import unittest
from types import SimpleNamespace

from mission_layout import MissionLayout
from route_follower import RouteFollower, mission_to_local, prepare_route
from DebugGUI import RobotDebugGUI


class RouteFollowerTests(unittest.TestCase):
    def test_gui_route_stop_sends_firmware_stop(self):
        sent = []
        fake = SimpleNamespace(
            route_follower=RouteFollower([(0, 1000)]),
            route_awaiting_pose=False,
            route_awaiting_limit=False,
            parameter_values={"drive.max_percent": 100},
            route_started_at=1.0,
            bluetooth=SimpleNamespace(is_connected=lambda: True,
                                      send_command=lambda name: sent.append(name)),
            recorder=SimpleNamespace(record_command=lambda *args: None),
            mission_view=SimpleNamespace(set_follow_status=lambda text: None),
            add_log=lambda *args: None,
        )
        RobotDebugGUI._stop_mission_route(fake, "test stop")
        self.assertEqual(sent, ["stop"])
        self.assertIsNone(fake.route_follower)

    def test_gui_route_tick_sends_drive_then_stops_on_stale_frame(self):
        sent = []
        stopped = []
        frame = {
            "encoder.0": 100, "encoder.1": -100,
            "imu.heading": 0, "imu.cal_gyro": 3,
            "tof.array_frame_ok": True,
            "tof.array.r2c3": 900,
        }
        fake = SimpleNamespace(
            route_follower=RouteFollower([(0, 1000)]),
            route_last_frame_monotonic=time.monotonic(),
            route_started_at=time.monotonic(),
            route_awaiting_pose=False,
            route_awaiting_limit=False,
            parameter_values={"drive.max_percent": 100},
            telemetry={},
            arena_view=SimpleNamespace(model=SimpleNamespace(latest=frame, x=0, y=0, theta=math.pi/2)),
            mission_view=SimpleNamespace(set_robot_pose=lambda *args: None,
                                         set_follow_status=lambda *args, **kwargs: None),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs: sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _stop_mission_route=lambda reason: stopped.append(reason),
        )
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1], (("drive_set",), {"left": 86, "right": 86}))
        fake.route_last_frame_monotonic = time.monotonic() - 2
        RobotDebugGUI._mission_route_step(fake)
        self.assertIn("Telemetry stale", stopped[-1])
        self.assertEqual(len(sent), 1)

    def test_forward_gate_ignores_floor_rows_but_fails_without_a_return(self):
        self.assertIsNone(RobotDebugGUI._route_front_range({"tof.array.r7c3": 25}))
        self.assertEqual(RobotDebugGUI._route_front_range({
            "tof.array.r7c3": 25, "tof.array.r2c3": 900,
            "tof.array.r3c4": 1200,
        }), 900)

    def test_mission_coordinates_rotate_into_robot_frame(self):
        self.assertEqual(mission_to_local((325, 325), 0, (1325, 325)), (0, 1000))
        right, forward = mission_to_local((325, 325), 90, (1325, 325))
        self.assertAlmostEqual(right, -1000)
        self.assertAlmostEqual(forward, 0, places=6)

    def test_prepared_weight_route_is_bounded_and_clear(self):
        layout = MissionLayout()
        layout.weights.append({"x": 1600, "y": 1100, "dummy": False})
        self.assertTrue(layout.plan(), layout.error)
        points = prepare_route(layout)
        self.assertGreater(len(points), 0)
        self.assertLessEqual(len(points), 64)

    def test_turn_then_advance_and_finish(self):
        follower = RouteFollower([(0, 1000)])
        forward = follower.step(0, 0, math.pi/2, 1200, 0)
        self.assertEqual(forward.state, "FORWARD")
        self.assertEqual((forward.left, forward.right), (86, 86))
        follower.step(0, 320, math.pi/2, 1200, 1)
        follower.step(0, 640, math.pi/2, 1200, 2)
        done = follower.step(0, 960, math.pi/2, 1200, 3)
        self.assertTrue(done.done)
        self.assertEqual((done.left, done.right), (0, 0))

        right = RouteFollower([(1000, 0)]).step(0, 0, math.pi/2, 1200, 0)
        self.assertEqual(right.state, "TURNING")
        self.assertGreater(right.left, 0)
        self.assertLess(right.right, 0)

    def test_every_moving_wheel_is_at_least_80_percent(self):
        for heading_error in (-7, -4, 0, 4, 7, 20, -90):
            theta = math.pi/2 - math.radians(heading_error)
            decision = RouteFollower([(0, 1000)]).step(0, 0, theta, 1200, 0)
            self.assertFalse(decision.fault)
            self.assertTrue(all(80 <= abs(v) <= 100 for v in (decision.left, decision.right)))

    def test_gui_waits_for_drive_limit_confirmation(self):
        sent = []
        stopped = []
        fake = SimpleNamespace(
            route_follower=RouteFollower([(0, 1000)]),
            route_started_at=time.monotonic(),
            route_awaiting_limit=True,
            parameter_values={},
            telemetry={},
            _drive_controls_available=lambda: True,
            _stop_mission_route=lambda reason: stopped.append(reason),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs: sent.append(args)),
        )
        RobotDebugGUI._mission_route_step(fake)
        self.assertFalse(sent)
        fake.route_started_at -= 3
        RobotDebugGUI._mission_route_step(fake)
        self.assertIn("not confirmed", stopped[-1])

    def test_obstacle_missing_range_off_route_and_heading_jump_stop(self):
        clear = RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, None, 0)
        self.assertEqual(clear.state, "FORWARD")
        self.assertTrue(RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 700, 0).fault)
        self.assertTrue(RouteFollower([(0, 1000)]).step(300, 0, math.pi/2, 1200, 0).fault)
        follower = RouteFollower([(0, 1000)])
        follower.step(0, 0, math.pi/2, 1200, 0)
        self.assertTrue(follower.step(0, 0, math.pi/2 + math.radians(50), 1200, 1).fault)

    def test_gui_stops_on_failed_8x8_frame_even_if_zones_are_empty(self):
        sent = []
        stopped = []
        frame = {"encoder.0": 10, "encoder.1": -10,
                 "imu.heading": 0, "imu.cal_gyro": 3,
                 "tof.array_frame_ok": False}
        fake = SimpleNamespace(
            route_follower=RouteFollower([(0, 1000)]),
            route_last_frame_monotonic=time.monotonic(),
            route_started_at=time.monotonic(),
            route_awaiting_pose=False,
            route_awaiting_limit=False,
            parameter_values={"drive.max_percent": 100},
            telemetry={},
            arena_view=SimpleNamespace(model=SimpleNamespace(latest=frame)),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs: sent.append(args)),
            _drive_controls_available=lambda: True,
            _stop_mission_route=lambda reason: stopped.append(reason),
        )
        RobotDebugGUI._mission_route_step(fake)
        self.assertIn("8×8", stopped[-1])
        self.assertFalse(sent)

    def test_gui_drives_with_healthy_empty_8x8_frame(self):
        sent = []
        frame = {"encoder.0": 10, "encoder.1": -10,
                 "imu.heading": 0, "imu.cal_gyro": 3,
                 "tof.array_frame_ok": True,
                 "tof.array_valid_zones": 0}
        fake = SimpleNamespace(
            route_follower=RouteFollower([(0, 1000)]),
            route_last_frame_monotonic=time.monotonic(),
            route_started_at=time.monotonic(),
            route_awaiting_pose=False,
            route_awaiting_limit=False,
            parameter_values={"drive.max_percent": 100},
            telemetry={},
            arena_view=SimpleNamespace(model=SimpleNamespace(latest=frame, x=0, y=0, theta=math.pi/2)),
            mission_view=SimpleNamespace(set_robot_pose=lambda *args: None,
                                         set_follow_status=lambda *args, **kwargs: None),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs: sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _stop_mission_route=lambda reason: self.fail(reason),
        )
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent, [(("drive_set",), {"left": 86, "right": 86})])

    def test_no_progress_times_out(self):
        follower = RouteFollower([(0, 1000)])
        follower.step(0, 0, math.pi/2, 1200, 0)
        self.assertTrue(follower.step(0, 0, math.pi/2, 1200, 9).fault)


if __name__ == "__main__":
    unittest.main()
