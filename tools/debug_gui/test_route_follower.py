"""Offline route-following regressions; these tests never command hardware."""

import math
import time
import unittest
from types import SimpleNamespace

from mission_layout import MissionLayout
from route_follower import (RouteFollower, local_to_mission,
                            matrix_obstacle_points, mission_to_local, prepare_route)
from DebugGUI import RobotDebugGUI
from arena_view import ArenaModel


class RouteFollowerTests(unittest.TestCase):
    def test_gui_route_stop_sends_firmware_stop(self):
        sent = []
        fake = SimpleNamespace(
            route_follower=RouteFollower([(0, 1000)]),
            route_awaiting_pose=False,
            route_awaiting_limit=False,
            parameter_values={"drive.max_percent": 100},
            route_pause_until=None,
            route_resume_after_frame=None,
            route_last_obstacle_frame=time.monotonic(),
            route_started_at=1.0,
            bluetooth=SimpleNamespace(is_connected=lambda: True,
                                      send_command=lambda name: sent.append(name)),
            recorder=SimpleNamespace(record_command=lambda *args: None,
                                     record_log=lambda *args: None),
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
            route_pause_until=None,
            route_resume_after_frame=None,
            telemetry={},
            arena_view=SimpleNamespace(model=SimpleNamespace(latest=frame, x=0, y=0, theta=math.pi/2)),
            mission_view=SimpleNamespace(set_robot_pose=lambda *args: None,
                                         set_follow_status=lambda *args, **kwargs: None),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs: sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None,
                                     record_log=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _stop_mission_route=lambda reason: stopped.append(reason),
        )
        fake.route_last_obstacle_frame = fake.route_last_frame_monotonic
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1], (("drive_set",), {"left": 85, "right": 100}))
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
        self.assertEqual(local_to_mission((325, 325), 0, (0, 1000)), (1325, 325))

    def test_8x8_obstacle_projects_to_map_and_detours(self):
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": True}]
        arena = ArenaModel()
        arena.sensor_specs = [{"kind": "matrix", "x": 0, "y": 0,
                               "angle": 0, "enabled": True}]
        frame = {"tof.array.r1c3": 500, "tof.array.r2c3": 500}
        points = matrix_obstacle_points(frame, arena, layout)
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 825, delta=20)
        layout.add_live_obstacles(points)
        self.assertFalse(layout.route_is_clear_from(layout.start, layout.route))
        detour = layout.replan_from(layout.start, layout.route)
        self.assertTrue(layout.route_is_clear_from(layout.start, detour))
        self.assertEqual(detour[-1]["x"], 1600)

    def test_8x8_obstacle_off_path_does_not_force_detour(self):
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": True}]
        layout.add_live_obstacles([(825, 1350)])
        self.assertTrue(layout.route_is_clear_from(layout.start, layout.route))

    def test_isolated_8x8_pixel_is_not_mapped_as_obstacle(self):
        layout = MissionLayout()
        arena = ArenaModel()
        arena.sensor_specs = [{"kind": "matrix", "x": 0, "y": 0,
                               "angle": 0, "enabled": True}]
        self.assertEqual(matrix_obstacle_points({"tof.array.r2c3": 250}, arena, layout), [])
        self.assertEqual(RobotDebugGUI._route_front_range({"tof.array.r2c3": 250}), 250)

    def test_blocked_goal_is_not_silently_skipped(self):
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": True}]
        layout.add_live_obstacles([(1600, 325)])
        with self.assertRaisesRegex(ValueError, "target is blocked"):
            layout.replan_from(layout.start, layout.route)

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
        self.assertEqual((forward.left, forward.right), (85, 100))
        follower.step(0, 320, math.pi/2, 1200, 1)
        follower.step(0, 640, math.pi/2, 1200, 2)
        done = follower.step(0, 960, math.pi/2, 1200, 3)
        self.assertTrue(done.done)
        self.assertEqual((done.left, done.right), (0, 0))

        right = RouteFollower([(1000, 0)]).step(0, 0, math.pi/2, 1200, 0)
        self.assertEqual(right.state, "TURNING")
        self.assertGreater(right.left, 0)
        self.assertLess(right.right, 0)

    def test_reaches_target_site_search_stops_then_advances_without_false_pickup(self):
        follower = RouteFollower([(0, 0), (140, -80), (140, 80), (0, 800)])
        scan = follower.step(0, 0, math.pi / 2, None, 1,
                             target_leg=True, search_waypoint=True)
        self.assertEqual(scan.state, "SITE_SEARCH")
        self.assertEqual((scan.left, scan.right), (0, 0))
        self.assertEqual(follower.index, 0)
        next_view = follower.step(0, 0, math.pi / 2, None, 1.1,
                                  target_leg=True, search_waypoint=True)
        self.assertFalse(next_view.fault)
        self.assertFalse(next_view.done)
        self.assertEqual(follower.index, 0)  # hold the view for the dwell
        follower.step(0, 0, math.pi / 2, None, 1.9,
                      target_leg=True, search_waypoint=True)
        self.assertEqual(follower.index, 1)
        move_to_view = follower.step(0, 0, math.pi / 2, None, 2.0,
                                     target_leg=True, search_waypoint=True)
        self.assertGreaterEqual(abs(move_to_view.left), 80)
        self.assertGreaterEqual(abs(move_to_view.right), 80)

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
        self.assertTrue(RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 300, 0).fault)
        self.assertFalse(RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 500, 0).fault)
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
            route_pause_until=None,
            route_resume_after_frame=None,
            telemetry={},
            arena_view=SimpleNamespace(model=SimpleNamespace(latest=frame)),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs: sent.append(args)),
            _drive_controls_available=lambda: True,
            _stop_mission_route=lambda reason: stopped.append(reason),
        )
        RobotDebugGUI._mission_route_step(fake)
        self.assertIn("8×8", stopped[-1])
        self.assertFalse(sent)

    def test_gui_keeps_following_when_nearby_8x8_return_cannot_be_mapped(self):
        sent = []
        frame = {"encoder.0": 10, "encoder.1": -10,
                 "imu.heading": 0, "imu.cal_gyro": 3,
                 "tof.array_frame_ok": True,
                 "tof.array_valid_zones": 1,
                 "tof.array.r4c2": 500}
        layout = MissionLayout()
        layout.route = [{"x": 1325, "y": 325, "target": False}]
        arena = ArenaModel()
        arena.sensor_specs = [{"kind": "matrix", "x": 0, "y": 0,
                               "angle": 0, "enabled": True}]
        arena.latest = frame
        fake = SimpleNamespace(
            route_follower=RouteFollower([(0, 1000)]),
            route_last_frame_monotonic=time.monotonic(),
            route_started_at=time.monotonic(),
            route_awaiting_pose=False,
            route_awaiting_limit=False,
            parameter_values={"drive.max_percent": 100},
            route_pause_until=None,
            route_resume_after_frame=None,
            telemetry={},
            arena_view=SimpleNamespace(model=arena),
            mission_view=SimpleNamespace(set_robot_pose=lambda *args: None,
                                         model=layout,
                                         set_follow_status=lambda *args, **kwargs: None),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs: sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None,
                                     record_log=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _stop_mission_route=lambda reason: self.fail(reason),
        )
        fake.route_last_obstacle_frame = None
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent, [(("drive_set",), {"left": 85, "right": 100})])

    def test_gui_stops_then_replans_around_500mm_obstacle(self):
        sent = []
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": True}]
        arena = ArenaModel()
        arena.sensor_specs = [{"kind": "matrix", "x": 0, "y": 0,
                               "angle": 0, "enabled": True}]
        arena.latest = {"encoder.0": 10, "encoder.1": -10,
                        "imu.heading": 0, "imu.cal_gyro": 3,
                        "tof.array_frame_ok": True,
                        "tof.array.r1c3": 500, "tof.array.r2c3": 500}
        fake = SimpleNamespace(
            route_follower=RouteFollower(prepare_route(layout)),
            route_last_frame_monotonic=time.monotonic(),
            route_last_obstacle_frame=None,
            route_started_at=time.monotonic(),
            route_awaiting_pose=False,
            route_awaiting_limit=False,
            route_pause_until=None,
            route_resume_after_frame=None,
            route_replans=0,
            parameter_values={"drive.max_percent": 100},
            telemetry={},
            arena_view=SimpleNamespace(model=arena),
            mission_view=SimpleNamespace(
                model=layout, canvas=SimpleNamespace(update=lambda: None),
                refresh_live_obstacles=lambda: None,
                set_follow_status=lambda *args, **kwargs: None,
            ),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs: sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None,
                                     record_log=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _stop_mission_route=lambda reason: self.fail(reason),
            add_log=lambda *args: None,
        )
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent, [(("stop",), {})])
        self.assertEqual(fake.route_replans, 1)
        self.assertGreater(len(layout.route), 1)
        fake.route_pause_until = time.monotonic() - 1
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent, [(("stop",), {})])  # same old scan cannot restart the motors
        fake.route_last_frame_monotonic = time.monotonic()
        fake.mission_view.set_robot_pose = lambda *args: None
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1][0], ("drive_set",))

    def test_no_progress_times_out(self):
        follower = RouteFollower([(0, 1000)])
        follower.step(0, 0, math.pi/2, 1200, 0)
        self.assertTrue(follower.step(0, 0, math.pi/2, 1200, 9).fault)


if __name__ == "__main__":
    unittest.main()
