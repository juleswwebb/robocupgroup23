"""Offline regression tests: run with python -m unittest test_arena_planner."""

import math
import unittest

from arena_planner import plan_route, point_segment_distance


class PlannerTests(unittest.TestCase):
    def test_free_space(self):
        route = plan_route((0, 0), (400, 0))
        self.assertEqual(route[0], (0, 0))
        self.assertEqual(route[-1], (400, 0))

    def test_obstacle_detour_respects_inflation(self):
        route = plan_route((0, 0), (800, 0), obstacles=[(400, 0, 100)])
        self.assertTrue(route)
        self.assertGreater(max(abs(y) for _, y in route), 0)
        self.assertTrue(all(math.hypot(x - 400, y) > 320 for x, y in route))

    def test_wall_cannot_be_crossed(self):
        self.assertEqual([], plan_route((0, 0), (600, 0),
                                        walls=[(300, -3000, 300, 3000)]))

    def test_blocked_start_or_goal(self):
        self.assertEqual([], plan_route((0, 0), (700, 0), obstacles=[(0, 0, 100)]))
        self.assertEqual([], plan_route((0, 0), (700, 0), obstacles=[(700, 0, 100)]))

    def test_segment_distance(self):
        self.assertEqual(point_segment_distance(5, 3, 0, 0, 10, 0), 3)


if __name__ == "__main__":
    unittest.main()
