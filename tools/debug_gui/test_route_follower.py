"""Offline route-following regressions; these tests never command hardware."""

import math
import time
import unittest
from types import SimpleNamespace

from mission_layout import MissionLayout
from route_follower import (RouteFollower, local_to_mission,
                            PointTofStopGuard,
                            confirm_obstacle_points, filter_uncorroborated_front_tofs,
                            matrix_confirms_near_front, matrix_obstacle_points,
                            mission_to_local, point_tof_obstacle_points,
                            prepare_route)
from DebugGUI import RobotDebugGUI
from arena_view import ArenaModel


class RouteFollowerTests(unittest.TestCase):
    def test_front_top_tofs_need_a_coherent_upper_array_pair_for_emergency_vote(self):
        point_readings = {"xshut7": 12, "xshut8": 18, "xshut5": 10}
        floor_only = {"tof.array.r5c3": 12, "tof.array.r6c3": 18}
        self.assertFalse(matrix_confirms_near_front(floor_only))
        self.assertEqual(
            filter_uncorroborated_front_tofs(point_readings, floor_only),
            {"xshut5": 10},
        )

        isolated_pixel = {"tof.array.r2c3": 110}
        self.assertFalse(matrix_confirms_near_front(isolated_pixel))

        coherent_pair = {"tof.array.r2c3": 110, "tof.array.r2c4": 145}
        self.assertTrue(matrix_confirms_near_front(coherent_pair))
        self.assertEqual(
            filter_uncorroborated_front_tofs(point_readings, coherent_pair),
            point_readings,
        )

        disagreeing_pair = {"tof.array.r2c3": 55, "tof.array.r2c4": 400}
        self.assertFalse(matrix_confirms_near_front(disagreeing_pair))

    def test_point_tof_guard_holds_first_sample_and_requires_persistent_hit(self):
        guard = PointTofStopGuard(required_frames=2)
        self.assertEqual(guard.observe(("Front_Top_Right", 288), 1, threshold_mm=300), "pending")
        self.assertEqual(guard.observe(("Front_Top_Right", 288), 1, threshold_mm=300), "pending")
        self.assertEqual(guard.observe(("Front_Top_Right", 476), 2, threshold_mm=300), "pending")
        self.assertEqual(guard.observe(("Front_Top_Right", 476), 3, threshold_mm=300), "clear")
        self.assertEqual(guard.observe(("Front_Top_Right", 288), 4, threshold_mm=300), "pending")
        self.assertEqual(guard.observe(("Front_Top_Right", 288), 5, threshold_mm=300), "confirmed")
        self.assertEqual(guard.observe(("Front_Top_Right", 288), 5, threshold_mm=300), "confirmed")
        self.assertEqual(guard.observe(("Front_Top_Right", 0), 6, threshold_mm=0), "clear")

    def test_obstacle_cutoff_is_strictly_below_50_mm(self):
        guard = PointTofStopGuard(required_frames=2)
        self.assertEqual(guard.observe({"xshut4": 50}, 1), "clear")
        self.assertEqual(guard.observe({"xshut4": 49}, 2), "pending")
        self.assertEqual(guard.observe({"xshut4": 50}, 3), "pending")
        self.assertEqual(guard.observe({"xshut4": 50}, 4), "clear")
        self.assertEqual(guard.observe({"xshut4": 49}, 5), "pending")
        self.assertEqual(guard.observe({"xshut4": 49}, 6), "confirmed")

        clear_follower = RouteFollower([(0, 1000)])
        self.assertNotEqual(clear_follower.step(0, 0, 0, 50, 0).state, "FAULT")
        close_follower = RouteFollower([(0, 1000)])
        self.assertEqual(close_follower.step(0, 0, 0, 49, 0).state, "FAULT")

    def test_point_tof_hold_requires_same_sensor_to_clear(self):
        guard = PointTofStopGuard(required_frames=2)
        self.assertEqual(guard.observe({"xshut6": 15}, 1), "pending")
        self.assertEqual(guard.observe({"xshut3": 500}, 2), "pending")
        self.assertEqual(guard.observe({}, 3), "pending")
        self.assertEqual(guard.observe({"xshut6": 100}, 4), "pending")
        self.assertEqual(guard.observe({"xshut6": 100}, 5), "clear")

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
        self.assertEqual(sent[-1], (("drive_set",), {"left": 100, "right": 85}))
        fake.route_last_frame_monotonic = time.monotonic() - 2
        RobotDebugGUI._mission_route_step(fake)
        self.assertIn("Telemetry stale", stopped[-1])
        self.assertEqual(len(sent), 1)

    def test_gui_route_keeps_driving_when_pose_is_only_1mm_inside_edge_margin(self):
        sent = []
        now = time.monotonic()
        layout = MissionLayout()
        layout.start = (1000, 307)
        layout.route = [{"x": 1500, "y": 400, "target": False}]
        model = ArenaModel()
        model.x, model.y, model.theta = -3, 100, math.pi / 2
        model.latest = {
            "encoder.0": 100, "encoder.1": -100,
            "imu.heading": 0, "imu.cal_gyro": 3,
            "tof.array_frame_ok": True,
        }
        fake = SimpleNamespace(
            route_follower=RouteFollower(prepare_route(layout)),
            route_last_frame_monotonic=now, route_last_obstacle_frame=None,
            route_previous_obstacle_points=[], route_started_at=now,
            route_awaiting_pose=False, route_awaiting_limit=False,
            route_pause_until=None, route_resume_after_frame=None, route_replans=0,
            route_weight_candidate_count=0,
            parameter_values={"drive.max_percent": 100}, telemetry={},
            arena_view=SimpleNamespace(model=model),
            mission_view=SimpleNamespace(
                model=layout, canvas=SimpleNamespace(update=lambda: None),
                refresh_live_obstacles=lambda: None,
                set_robot_pose=lambda *args: None,
                set_follow_status=lambda *args, **kwargs: None,
            ),
            bluetooth=SimpleNamespace(send_command=lambda *args, **kwargs:
                                      sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None,
                                     record_log=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _route_point_tof_range=RobotDebugGUI._route_point_tof_range,
            _stop_mission_route=lambda reason, **kwargs: self.fail(reason),
            add_log=lambda *args: None,
        )
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1][0], ("drive_set",))
        self.assertEqual(fake.route_replans, 0)

    def test_forward_gate_ignores_floor_rows_but_fails_without_a_return(self):
        self.assertIsNone(RobotDebugGUI._route_front_range({"tof.array.r7c3": 25}))
        self.assertIsNone(RobotDebugGUI._route_front_range({"tof.array.r4c3": 430}))
        self.assertEqual(RobotDebugGUI._route_front_range({"tof.array.r3c3": 430}), 430)
        self.assertEqual(RobotDebugGUI._route_front_range({
            "tof.array.r7c3": 25, "tof.array.r2c3": 900,
            "tof.array.r3c4": 1200,
        }), 900)

    def test_vl53_point_sensors_are_projected_and_saturated_values_ignored(self):
        layout = MissionLayout()
        arena = ArenaModel()
        arena.sensor_specs = [
            {"key": "xshut8", "name": "Top_Left", "signal": "tof.xshut8",
             "kind": "point", "x": -90, "y": 140, "angle": 0, "enabled": True},
            {"key": "xshut7", "name": "Top_Right", "signal": "tof.xshut7",
             "kind": "point", "x": 90, "y": 140, "angle": 0, "enabled": True},
        ]
        frame = {"tof.xshut8": 20, "tof.xshut7": 8191}
        points = point_tof_obstacle_points(frame, arena, layout)
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 485, delta=1)
        self.assertAlmostEqual(points[0][1], 235, delta=1)
        excluded = point_tof_obstacle_points(
            frame, arena, layout, excluded_keys={"xshut8"},
        )
        self.assertEqual(excluded, [])
        boundary = point_tof_obstacle_points(
            {"tof.xshut8": 50}, arena, layout,
        )
        self.assertEqual(boundary, [])
        just_inside = point_tof_obstacle_points(
            {"tof.xshut8": 49}, arena, layout,
        )
        self.assertEqual(len(just_inside), 1)

    def test_all_six_wired_point_tofs_reach_navigation_by_signal_id(self):
        arena = ArenaModel()
        arena.sensor_specs = [
            {"key": f"xshut{n}", "name": f"Sensor {n}",
             "signal": f"tof.xshut{n}", "kind": "point", "enabled": True,
             "x": 0, "y": 140, "angle": 0}
            for n in range(3, 9)
        ]
        frame = {f"tof.xshut{n}": 100 + n for n in range(3, 9)}
        frame["ultrasonic.0"] = 150
        readings = RobotDebugGUI._route_point_tof_readings(frame, arena)
        self.assertEqual(readings,
                         {f"xshut{n}": 100 + n for n in range(3, 9)})
        arena.sensor_specs[-1]["enabled"] = False
        self.assertNotIn("xshut8", RobotDebugGUI._route_point_tof_readings(frame, arena))

    def test_sensor_obstacle_point_requires_three_nearby_frames(self):
        self.assertEqual(confirm_obstacle_points([(800, 900)], []), [])
        self.assertEqual(confirm_obstacle_points([(805, 900)], [(800, 900)]), [])
        self.assertEqual(confirm_obstacle_points(
            [(810, 900)], [(805, 900)], [(800, 900)],
        ), [(810, 900)])
        self.assertEqual(confirm_obstacle_points(
            [(1100, 900)], [(805, 900)], [(800, 900)],
        ), [])

    def test_gui_ignores_long_range_point_tof_returns_for_obstacle_mapping(self):
        sent = []
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": True}]
        arena = ArenaModel()
        arena.sensor_specs = [{
            "key": "xshut8", "name": "Top_Left", "signal": "tof.xshut8",
            "kind": "point", "x": -90, "y": 140, "angle": 0, "enabled": True,
        }]
        arena.latest = {"encoder.0": 10, "encoder.1": -10,
                        "imu.heading": 0, "imu.cal_gyro": 3,
                        "tof.array_frame_ok": True, "tof.xshut8": 430}
        fake = SimpleNamespace(
            route_follower=RouteFollower(prepare_route(layout)),
            route_last_frame_monotonic=time.monotonic(), route_last_obstacle_frame=None,
            route_previous_obstacle_points=[], route_started_at=time.monotonic(),
            route_awaiting_pose=False, route_awaiting_limit=False,
            route_pause_until=None, route_resume_after_frame=None, route_replans=0,
            route_weight_candidate_count=0,
            parameter_values={"drive.max_percent": 100}, telemetry={},
            arena_view=SimpleNamespace(model=arena),
            mission_view=SimpleNamespace(
                model=layout, canvas=SimpleNamespace(update=lambda: None),
                refresh_live_obstacles=lambda: None, set_robot_pose=lambda *args: None,
                set_follow_status=lambda *args, **kwargs: None,
            ),
            bluetooth=SimpleNamespace(
                send_command=lambda *args, **kwargs: sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None,
                                     record_log=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _stop_mission_route=lambda reason: self.fail(reason),
            add_log=lambda *args: None,
        )
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1][0], ("drive_set",))
        fake.route_last_frame_monotonic = time.monotonic()
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1][0], ("drive_set",))
        fake.route_last_frame_monotonic = time.monotonic()
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1][0], ("drive_set",))
        self.assertEqual(fake.route_replans, 0)
        self.assertEqual(layout.live_obstacles, [])

    def test_weight_pair_is_not_mapped_as_two_generic_obstacles(self):
        sent = []
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": True}]
        arena = ArenaModel()
        arena.sensor_specs = [
            {"key": "xshut6", "name": "Front_Top_Left", "signal": "tof.xshut6",
             "kind": "point", "x": -75, "y": 140, "angle": 45, "enabled": True},
            {"key": "xshut5", "name": "Front_Bottom_Left", "signal": "tof.xshut5",
             "kind": "point", "x": -75, "y": 140, "angle": 45, "enabled": True},
        ]
        arena.weight_votes = {"xshut6": 1}
        arena.latest = {"encoder.0": 10, "encoder.1": -10,
                        "imu.heading": 0, "imu.cal_gyro": 3,
                        "tof.array_frame_ok": True,
                        "tof.xshut6": 850, "tof.xshut5": 470}
        fake = SimpleNamespace(
            route_follower=RouteFollower(prepare_route(layout)),
            route_last_frame_monotonic=time.monotonic(), route_last_obstacle_frame=None,
            route_previous_obstacle_points=[], route_older_obstacle_points=[],
            route_started_at=time.monotonic(), route_awaiting_pose=False,
            route_awaiting_limit=False, route_pause_until=None,
            route_resume_after_frame=None, route_replans=0,
            route_weight_candidate_count=0,
            parameter_values={"drive.max_percent": 100}, telemetry={},
            arena_view=SimpleNamespace(model=arena),
            mission_view=SimpleNamespace(
                model=layout, canvas=SimpleNamespace(update=lambda: None),
                refresh_live_obstacles=lambda: None, set_robot_pose=lambda *args: None,
                set_follow_status=lambda *args, **kwargs: None,
            ),
            bluetooth=SimpleNamespace(
                send_command=lambda *args, **kwargs: sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None,
                                     record_log=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _stop_mission_route=lambda reason: self.fail(reason),
            add_log=lambda *args: None,
        )
        for _ in range(4):
            fake.route_last_frame_monotonic = time.monotonic()
            RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(fake.route_replans, 0)
        self.assertEqual(layout.live_obstacles, [])
        self.assertEqual(sent[-1][0], ("drive_set",))

    def test_mission_coordinates_rotate_into_robot_frame(self):
        self.assertEqual(mission_to_local((325, 325), 0, (1325, 325)), (0, 1000))
        right, forward = mission_to_local((325, 325), 90, (1325, 325))
        self.assertAlmostEqual(right, -1000)
        self.assertAlmostEqual(forward, 0, places=6)
        self.assertEqual(local_to_mission((325, 325), 0, (0, 1000)), (1325, 325))

    def test_8x8_long_range_returns_are_not_mapped_as_obstacles(self):
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": True}]
        arena = ArenaModel()
        arena.sensor_specs = [{"kind": "matrix", "x": 0, "y": 0,
                               "angle": 0, "enabled": True}]
        frame = {"tof.array.r1c3": 500, "tof.array.r2c3": 500}
        points = matrix_obstacle_points(frame, arena, layout)
        self.assertEqual(points, [])
        points = matrix_obstacle_points({"tof.array.r1c3": 20}, arena, layout)
        self.assertEqual(len(points), 1)
        self.assertTrue(all(math.isfinite(value) for value in points[0]))
        self.assertEqual(matrix_obstacle_points({"tof.array.r1c3": 50}, arena, layout), [])
        self.assertEqual(len(matrix_obstacle_points(
            {"tof.array.r1c3": 49}, arena, layout)), 1)

    def test_8x8_obstacle_off_path_does_not_force_detour(self):
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": True}]
        layout.add_live_obstacles([(825, 1350)])
        self.assertTrue(layout.route_is_clear_from(layout.start, layout.route))

    def test_isolated_8x8_pixel_needs_a_repeat_before_mapping(self):
        layout = MissionLayout()
        arena = ArenaModel()
        arena.sensor_specs = [{"kind": "matrix", "x": 0, "y": 0,
                               "angle": 0, "enabled": True}]
        self.assertEqual(matrix_obstacle_points({"tof.array.r2c3": 250}, arena, layout), [])
        isolated = matrix_obstacle_points({"tof.array.r2c3": 20}, arena, layout)
        self.assertEqual(len(isolated), 1)
        self.assertEqual(confirm_obstacle_points(isolated, []), [])
        self.assertEqual(confirm_obstacle_points(isolated, isolated), [])
        self.assertEqual(confirm_obstacle_points(isolated, isolated, isolated), isolated)
        self.assertEqual(matrix_obstacle_points({"tof.array.r4c3": 520}, arena, layout), [])
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
        self.assertEqual((forward.left, forward.right), (100, 85))
        follower.step(0, 320, math.pi/2, 1200, 1)
        follower.step(0, 640, math.pi/2, 1200, 2)
        done = follower.step(0, 960, math.pi/2, 1200, 3)
        self.assertTrue(done.done)
        self.assertEqual((done.left, done.right), (0, 0))

        right = RouteFollower([(1000, 0)]).step(0, 0, math.pi/2, 1200, 0)
        self.assertEqual(right.state, "TURNING")
        self.assertGreater(right.left, 0)
        self.assertLess(right.right, 0)

    def test_50mm_threshold_only_hard_stops_below_50mm(self):
        at_detour_range = RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 430, 0)
        self.assertFalse(at_detour_range.fault)
        under_threshold = RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 49, 0)
        at_threshold = RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 50, 0)
        above_threshold = RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 51, 0)
        self.assertTrue(under_threshold.fault)
        self.assertFalse(at_threshold.fault)
        self.assertFalse(above_threshold.fault)

    def test_route_turns_use_hysteresis_instead_of_chattering_at_one_angle(self):
        follower = RouteFollower([(1000, 0)])
        within_forward_band = follower.step(
            0, 0, -math.radians(18), None, 0,
        )
        self.assertEqual(within_forward_band.state, "FORWARD")
        enter_turn = follower.step(0, 0, -math.radians(25), None, 0.1)
        self.assertEqual(enter_turn.state, "TURNING")
        remain_turning = follower.step(0, 0, -math.radians(18), None, 0.2)
        self.assertEqual(remain_turning.state, "TURNING")
        resume_forward = follower.step(0, 0, -math.radians(11), None, 0.3)
        self.assertEqual(resume_forward.state, "FORWARD")

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
        self.assertFalse(RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 300, 0).fault)
        self.assertFalse(RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 299, 0).fault)
        self.assertTrue(RouteFollower([(0, 1000)]).step(0, 0, math.pi/2, 0, 0).fault)
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
        self.assertEqual(sent, [(("drive_set",), {"left": 100, "right": 85})])

    def test_close_point_tof_hold_needs_same_sensor_clearance_to_resume(self):
        sent = []
        layout = MissionLayout()
        layout.start = (1000, 1000)
        layout.route = [{"x": 1600, "y": 1000, "target": False}]
        arena = ArenaModel()
        arena.sensor_specs = [
            {"key": "xshut3", "name": "Front_Top_Right", "signal": "tof.xshut3",
             "kind": "point", "x": 0, "y": 0, "angle": 90, "enabled": True},
        ]
        frame = {"encoder.0": 10, "encoder.1": -10,
                 "imu.heading": 0, "imu.cal_gyro": 3,
                 "tof.array_frame_ok": True, "tof.xshut3": 15}
        arena.latest = frame
        now = time.monotonic()
        fake = SimpleNamespace(
            route_follower=RouteFollower(prepare_route(layout)),
            route_last_frame_monotonic=now, route_last_obstacle_frame=now,
            route_previous_obstacle_points=[], route_started_at=now,
            route_awaiting_pose=False, route_awaiting_limit=False,
            route_pause_until=None, route_resume_after_frame=None,
            route_replans=0, route_weight_candidate_count=0,
            route_last_replan_pose=None, route_clearance_hold=False,
            route_point_tof_guard=PointTofStopGuard(), route_point_tof_pending=False,
            parameter_values={"drive.max_percent": 100}, telemetry={},
            arena_view=SimpleNamespace(model=arena),
            mission_view=SimpleNamespace(
                model=layout, canvas=SimpleNamespace(update=lambda: None),
                refresh_live_obstacles=lambda: None,
                set_robot_pose=lambda *args: None,
                set_follow_status=lambda *args, **kwargs: None,
            ),
            bluetooth=SimpleNamespace(
                send_command=lambda *args, **kwargs: sent.append((args, kwargs))),
            recorder=SimpleNamespace(record_command=lambda *args, **kwargs: None,
                                     record_log=lambda *args, **kwargs: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            _route_point_tof_range=RobotDebugGUI._route_point_tof_range,
            _stop_mission_route=lambda reason: self.fail(reason),
            add_log=lambda *args: None,
        )

        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1][0], ("stop",))
        self.assertIsNotNone(fake.route_follower)

        fake.route_last_frame_monotonic = now + 0.1
        arena.latest = dict(frame, **{"tof.xshut3": 476})
        RobotDebugGUI._mission_route_step(fake)
        self.assertTrue(fake.route_point_tof_pending)
        self.assertIsNotNone(fake.route_follower)
        self.assertEqual(sent[-1][0], ("stop",))

        fake.route_last_frame_monotonic = now + 0.2
        RobotDebugGUI._mission_route_step(fake)
        self.assertFalse(fake.route_point_tof_pending)
        self.assertEqual(sent[-1][0], ("stop",))

        fake.route_last_frame_monotonic = now + 0.3
        RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1][0], ("drive_set",))

    def test_gui_does_not_map_or_stop_for_500mm_matrix_return(self):
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
                set_robot_pose=lambda *args: None,
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
        for _ in range(3):
            fake.route_last_frame_monotonic = time.monotonic()
            RobotDebugGUI._mission_route_step(fake)
        self.assertEqual(sent[-1][0], ("drive_set",))
        self.assertEqual(fake.route_replans, 0)
        self.assertEqual(layout.live_obstacles, [])

    def test_no_progress_times_out(self):
        follower = RouteFollower([(0, 1000)])
        follower.step(0, 0, math.pi/2, 1200, 0)
        self.assertTrue(follower.step(0, 0, math.pi/2, 1200, 9).fault)


if __name__ == "__main__":
    unittest.main()
