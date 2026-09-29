"""Group 23 offline mission-planning regression tests."""

import math
import unittest

from mission_layout import MissionLayout
from route_follower import prepare_route


class MissionTests(unittest.TestCase):
    def test_real_weight_and_return_home(self):
        mission = MissionLayout()
        mission.weights.append({"x": 1600, "y": 1100, "dummy": False})
        self.assertTrue(mission.plan(return_home=True), mission.error)
        self.assertTrue(any(point["target"] for point in mission.route))
        self.assertLessEqual(len(mission.route), 64)
        self.assertLess(mission.route[-1]["x"], 650)

    def test_dummy_is_no_go_and_not_target(self):
        mission = MissionLayout()
        mission.weights = [{"x": 1200, "y": 800, "dummy": True},
                           {"x": 2400, "y": 1200, "dummy": False}]
        self.assertTrue(mission.blocked(1200, 800))
        self.assertTrue(mission.plan(), mission.error)
        site_stops = [point for point in mission.route if point["site_search"]]
        self.assertGreaterEqual(len(site_stops), 2)
        self.assertTrue(all(point["target_x"] == 2400 and point["target_y"] == 1200
                            for point in site_stops))
        self.assertTrue(all(not mission.blocked(point["x"], point["y"]) for point in mission.route))

    def test_weight_route_stages_before_weight_and_adds_sensor_search_stops(self):
        mission = MissionLayout()
        mission.weights.append({"x": 1800, "y": 1100, "dummy": False})
        self.assertTrue(mission.plan(), mission.error)
        stops = [point for point in mission.route if point["site_search"]]
        self.assertEqual(len(stops), 3)
        target = (1800, 1100)
        staging = (stops[0]["x"], stops[0]["y"])
        self.assertGreaterEqual(math.dist(staging, target), 150)
        self.assertLessEqual(math.dist(staging, target), 310)
        self.assertGreater(math.dist((stops[1]["x"], stops[1]["y"]),
                                     (stops[2]["x"], stops[2]["y"])), 100)

    def test_opposite_home_forbidden(self):
        mission = MissionLayout()
        self.assertTrue(mission.blocked(325, 2075))
        self.assertEqual(mission.blocked_reason(325, 2075), "opposite-home exclusion zone")
        mission.my_home = "blue"
        self.assertTrue(mission.blocked(325, 325))

    def test_block_reason_identifies_tight_arena_edge_clearance(self):
        mission = MissionLayout()
        self.assertEqual(mission.blocked_reason(300, 500),
                         "left arena boundary (x=300 < 305 mm clearance)")
        self.assertIsNone(mission.blocked_reason(325, 500))

    def test_route_continues_away_from_margin_only_edge_overlap(self):
        mission = MissionLayout()
        self.assertTrue(mission.route_is_clear_from(
            (1000, 304), [{"x": 1500, "y": 400}]))
        self.assertFalse(mission.route_is_clear_from(
            (1000, 304), [{"x": 1500, "y": 250}]))

    def test_replan_can_escape_a_live_obstacle_margin_without_crossing_other_blocks(self):
        mission = MissionLayout()
        mission.live_obstacles.append({"x": 700, "y": 325, "radius": 120})
        self.assertTrue(mission.blocked(325, 325))  # inflated clearance overlaps pose
        route = mission.replan_from((325, 325), [{"x": 1600, "y": 800}])
        self.assertGreaterEqual(len(route), 2)
        self.assertTrue(route[0].get("escape"))
        self.assertFalse(mission.blocked(route[0]["x"], route[0]["y"]))
        self.assertTrue(mission.route_is_clear_from(
            (route[0]["x"], route[0]["y"]), route[1:]))
        mission.route = route
        self.assertTrue(prepare_route(
            mission, current_local=(0, 0), current_mission=(325, 325),
            allow_buffered_start=True,
        ))
        with self.assertRaisesRegex(ValueError, "Current route start"):
            prepare_route(mission, current_local=(0, 0), current_mission=(325, 325))

    def test_replan_can_escape_edge_margin_but_not_actual_live_overlap(self):
        mission = MissionLayout()
        route = mission.replan_from((300, 1000), [{"x": 1300, "y": 1000}])
        self.assertGreaterEqual(route[0]["x"], 305)

        mission.live_obstacles.append({"x": 500, "y": 1000, "radius": 120})
        with self.assertRaisesRegex(ValueError, "live sensor return"):
            mission.replan_from((300, 1000), [{"x": 1300, "y": 1000}])

        mission = MissionLayout()
        mission.obstacles.append({"kind": "box", "x": 300, "y": 1000,
                                  "width": 200, "height": 200, "rotation": 0})
        with self.assertRaisesRegex(ValueError, "arena boundary"):
            mission.replan_from((300, 1000), [{"x": 1300, "y": 1000}])

    def test_blocked_weight_rejected(self):
        mission = MissionLayout()
        mission.weights.append({"x": 1500, "y": 1000, "dummy": False})
        mission.obstacles.append({"kind": "tube", "x": 1500, "y": 1000,
                                  "width": 320, "height": 320, "rotation": 0})
        self.assertFalse(mission.plan())

    def test_arbitrary_wall_angle_drives_exact_collision_and_route_clearance(self):
        mission = MissionLayout()
        mission.robot_radius_mm = mission.margin_mm = 0
        wall = {"kind": "wall", "x": 2000, "y": 1200,
                "width": 600, "height": 130, "rotation": 45}
        mission.obstacles.append(wall)
        self.assertTrue(mission.blocked(2200, 1400))  # along the long axis
        self.assertFalse(mission.blocked(2250, 1200))  # inside AABB, outside rotated wall
        self.assertTrue(mission._segment_clear((2250, 1050), (2250, 1350)))
        mission.robot_radius_mm = 120
        self.assertTrue(mission.blocked(2250, 1200))  # inflated chassis clearance
        mission.robot_radius_mm = 0
        wall["rotation"] = 0
        self.assertFalse(mission._segment_clear((2250, 1050), (2250, 1350)))

    def test_rotated_wall_persists_with_legacy_right_angle_wall(self):
        mission = MissionLayout()
        mission.obstacles = [
            {"kind": "wall", "x": 1500, "y": 1000, "width": 600,
             "height": 130, "rotation": 37.5},
            {"kind": "wall", "x": 2500, "y": 1000, "width": 600,
             "height": 130, "rotation": 90},
        ]
        restored = MissionLayout()
        restored.load_dict(mission.to_dict())
        self.assertEqual([wall["rotation"] for wall in restored.obstacles], [37.5, 90])
        x0, y0, x1, y1 = restored.obstacle_rect(restored.obstacles[1])
        self.assertAlmostEqual(x1-x0, 130)
        self.assertAlmostEqual(y1-y0, 600)


if __name__ == "__main__":
    unittest.main()
