"""Offline target classification and centering safety tests."""

import math
import time
import unittest
from types import SimpleNamespace

from arena_view import ArenaModel
from mission_layout import MissionLayout
from route_follower import RouteFollower
from route_follower import PointTofStopGuard, prepare_route
from weight_targeting import WeightTracker
from DebugGUI import RobotDebugGUI


class WeightTargetingTests(unittest.TestCase):
    def setUp(self):
        self.arena = ArenaModel()
        self.arena.theta = math.pi / 2
        self.arena.sensor_specs = [
            {"key": key, "kind": "point", "enabled": True, "x": x, "y": 100, "angle": 0}
            for key, x in (("xshut6", -30), ("xshut5", -30),
                           ("xshut3", 30), ("xshut4", 30))
        ] + [{"kind": "matrix", "enabled": True, "x": 0, "y": 150, "angle": 0}]
        self.layout = MissionLayout()
        self.target = (625.0, 325.0)  # 300 mm straight ahead in local coordinates
        self.frame = {"tof.xshut6": 600, "tof.xshut5": 200,
                      "tof.xshut3": 600, "tof.xshut4": 200}

    def _third(self, frame=None, front=200):
        tracker = WeightTracker()
        for stamp in (1.0, 1.1, 1.2):
            result = tracker.observe(frame or self.frame, self.arena, self.layout,
                                     self.target, front, stamp)
        return result

    def test_three_confirmed_frames_and_mapped_target(self):
        tracker = WeightTracker()
        first = tracker.observe(self.frame, self.arena, self.layout, self.target, 200, 1)
        self.assertFalse(first.confirmed)
        self.assertTrue(first.pending)
        self.assertFalse(tracker.observe(self.frame, self.arena, self.layout,
                                         self.target, 200, 1).confirmed)
        tracker.observe(self.frame, self.arena, self.layout, self.target, 200, 1.1)
        third = tracker.observe(self.frame, self.arena, self.layout, self.target, 200, 1.2)
        self.assertTrue(third.confirmed)
        self.assertFalse(third.pending)
        self.assertTrue(third.centered)
        self.assertTrue(third.front_is_target)
        self.assertAlmostEqual(third.hit[0], self.target[0])

    def test_unmatched_wall_and_missing_top_never_confirm(self):
        self.assertFalse(self._third(front=700).front_is_target)
        self.assertFalse(self._third({"tof.xshut5": 200, "tof.xshut4": 200}).confirmed)
        far = (1200.0, 325.0)
        tracker = WeightTracker()
        for stamp in (1, 1.1, 1.2):
            evidence = tracker.observe(self.frame, self.arena, self.layout, far, 200, stamp)
        self.assertFalse(evidence.confirmed)

    def test_no_stale_votes_or_disabled_pair(self):
        tracker = WeightTracker()
        for stamp in (1, 1.1):
            tracker.observe(self.frame, self.arena, self.layout, self.target, 200, stamp)
        self.assertFalse(tracker.observe(self.frame, self.arena, self.layout,
                                         self.target, 200, 2.1).confirmed)
        self.arena.sensor_specs[1]["enabled"] = False
        self.arena.sensor_specs[3]["enabled"] = False
        self.assertFalse(self._third().confirmed)

    def test_pending_verification_has_one_second_limit(self):
        tracker = WeightTracker()
        weak = {"tof.xshut6": 600, "tof.xshut5": 200}
        first = tracker.observe(weak, self.arena, self.layout, self.target, 200, 1)
        self.assertTrue(first.pending)
        for tick in range(1, 14):
            frame = {} if tick % 2 else weak
            last = tracker.observe(frame, self.arena, self.layout,
                                   self.target, 200, 1 + tick * 0.1)
        self.assertFalse(last.pending)

    def test_centering_and_20mm_range_stop_threshold(self):
        frame = dict(self.frame, **{"tof.xshut5": 150})
        evidence = self._third(frame, front=200)
        self.assertTrue(evidence.confirmed)
        self.assertFalse(evidence.centered)
        follower = RouteFollower([(0, 300)])
        pulse = follower.step(0, 0, math.pi / 2, 200, 1, weight=evidence,
                              target_leg=True)
        self.assertEqual(pulse.state, "CENTERING")
        self.assertEqual((pulse.left, pulse.right), (80, -80))
        stop = follower.step(0, 0, math.pi / 2, 149, 1.1, weight=evidence)
        self.assertFalse(stop.fault)
        self.assertFalse(RouteFollower([(0, 300)]).step(0, 0, math.pi / 2,
                         430, 1).fault)  # far return triggers mapping/replanning
        self.assertTrue(RouteFollower([(0, 300)]).step(0, 0, math.pi / 2,
                        19, 1).fault)  # configured near range stops

    def test_confirmed_weight_uses_a_bounded_final_approach_from_search_pose(self):
        evidence = SimpleNamespace(confirmed=True, centered=False,
                                   local_target=(0, 300), left_mm=150,
                                   right_mm=None, front_is_target=True)
        follower = RouteFollower([(0, 0), (100, 0)])
        decision = follower.step(
            0, 0, math.pi / 2, 200, 1, weight=evidence,
            target_leg=True, search_waypoint=True)
        self.assertEqual(decision.state, "TARGET_SETTLE")
        decision = follower.step(
            0, 0, math.pi / 2, 200, 1.3, weight=evidence,
            target_leg=True, search_waypoint=True)
        self.assertEqual(decision.state, "FORWARD")
        self.assertEqual((decision.left, decision.right), (85, 100))

    def test_aligned_target_stops_without_claiming_pickup(self):
        evidence = self._third()
        decision = RouteFollower([(0, 300)]).step(0, 0, math.pi / 2,
                                                  200, 1, weight=evidence,
                                                  target_leg=True)
        self.assertTrue(decision.done)
        self.assertEqual((decision.left, decision.right), (0, 0))
        self.assertIn("pickup unverified", decision.detail)
        missing = RouteFollower([(0, 300)]).step(0, 280, math.pi / 2,
                                                None, 1, target_leg=True)
        self.assertTrue(missing.fault)
        self.assertIn("without sensor-confirmed", missing.detail)

    def test_gui_stops_while_verifying_and_never_replans_over_weight(self):
        self.layout.route = [{"x": self.target[0], "y": self.target[1], "target": True}]
        self.arena.latest = dict(self.frame, **{
            "encoder.0": 0, "encoder.1": 0, "imu.heading": 0,
            "imu.cal_gyro": 3, "tof.array_frame_ok": True,
            "tof.array.r2c3": 200,
        })
        sent = []
        gui = SimpleNamespace(
            route_follower=RouteFollower([(0, 300)]),
            route_weight_tracker=WeightTracker(),
            route_started_at=time.monotonic(), route_awaiting_limit=False,
            route_awaiting_pose=False, route_last_obstacle_frame=None,
            route_pause_until=None, route_resume_after_frame=None,
            route_replans=0, parameter_values={"drive.max_percent": 100},
            telemetry={}, arena_view=SimpleNamespace(model=self.arena),
            mission_view=SimpleNamespace(
                model=self.layout, canvas=SimpleNamespace(update=lambda: None),
                set_follow_status=lambda *args, **kwargs: None,
                set_robot_pose=lambda *args: None,
                refresh_live_obstacles=lambda: None,
            ),
            bluetooth=SimpleNamespace(
                send_command=lambda *args, **kwargs: sent.append((args, kwargs)),
                is_connected=lambda: True,
            ),
            recorder=SimpleNamespace(record_command=lambda *args: None,
                                     record_log=lambda *args: None),
            _drive_controls_available=lambda: True,
            _route_front_range=RobotDebugGUI._route_front_range,
            add_log=lambda *args: None,
        )
        gui._stop_mission_route = lambda reason: RobotDebugGUI._stop_mission_route(gui, reason)
        for _ in range(3):
            gui.route_last_frame_monotonic = time.monotonic()
            RobotDebugGUI._mission_route_step(gui)
        self.assertIsNone(gui.route_follower)
        self.assertEqual(gui.route_replans, 0)
        self.assertEqual(self.layout.live_obstacles, [])
        self.assertEqual([args[0] for args, _ in sent], ["stop", "stop", "stop"])

    def test_unplanned_weight_signature_does_not_stop_transit(self):
        now = time.monotonic()
        sent = []
        layout = MissionLayout()
        layout.route = [{"x": 1600, "y": 325, "target": False}]
        arena = ArenaModel()
        arena.sensor_specs = [{"kind": "matrix", "key": "matrix",
                               "x": 0, "y": 150, "angle": 0, "enabled": True}]
        arena.latest = {
            "encoder.0": 10, "encoder.1": -10,
            "imu.heading": 0, "imu.cal_gyro": 3,
            "tof.array_frame_ok": True, "tof.array.r3c3": 1000,
        }
        arena.detected_weights = [(150.0, 90.0)]
        fake = SimpleNamespace(
            route_follower=RouteFollower(prepare_route(layout)),
            route_last_frame_monotonic=now,
            route_last_obstacle_frame=now,
            route_started_at=now,
            route_awaiting_pose=False, route_awaiting_limit=False,
            route_pause_until=None, route_resume_after_frame=None,
            route_replans=0, route_weight_candidate_count=0,
            route_point_tof_guard=PointTofStopGuard(), route_point_tof_pending=False,
            route_last_replan_pose=None, route_clearance_hold=False,
            parameter_values={"drive.max_percent": 100}, telemetry={},
            arena_view=SimpleNamespace(model=arena),
            mission_view=SimpleNamespace(
                model=layout, canvas=SimpleNamespace(update=lambda: None),
                set_robot_pose=lambda *args: None,
                set_follow_status=lambda *args, **kwargs: None,
                refresh_live_obstacles=lambda: None,
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

        self.assertIsNotNone(fake.route_follower)
        self.assertEqual(sent[-1][0], ("drive_set",))

    def test_final_leg_requires_heading_alignment_and_settle(self):
        follower = RouteFollower([(0, 600)])
        turn = follower.step(0, 0, math.pi / 2 - math.radians(30), None,
                             1, target_leg=True)
        self.assertEqual(turn.state, "TARGET_ALIGN")
        settle = follower.step(0, 0, math.pi / 2, None, 1.1,
                               target_leg=True)
        self.assertEqual(settle.state, "TARGET_SETTLE")
        still = follower.step(0, 0, math.pi / 2, None, 1.2,
                              target_leg=True)
        self.assertEqual((still.left, still.right), (0, 0))
        go = follower.step(0, 0, math.pi / 2, None, 1.36,
                           target_leg=True)
        self.assertEqual((go.left, go.right), (85, 100))


if __name__ == "__main__":
    unittest.main()
