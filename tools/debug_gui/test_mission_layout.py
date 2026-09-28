"""Group 23 offline mission-planning regression tests."""

import math
import unittest

from mission_layout import MissionLayout


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
        mission.my_home = "blue"
        self.assertTrue(mission.blocked(325, 325))

    def test_blocked_weight_rejected(self):
        mission = MissionLayout()
        mission.weights.append({"x": 1500, "y": 1000, "dummy": False})
        mission.obstacles.append({"kind": "tube", "x": 1500, "y": 1000,
                                  "width": 320, "height": 320, "rotation": 0})
        self.assertFalse(mission.plan())


if __name__ == "__main__":
    unittest.main()
