"""Regression checks for Group 23's wired top/bottom TOF pairs."""

import unittest

from arena_view import ArenaModel


class WeightTests(unittest.TestCase):
    def setUp(self):
        self.model = ArenaModel()
        self.model.sensor_specs = [
            {"key": "xshut6", "name": "Front_Top_Left", "signal": "tof.xshut6",
             "kind": "point", "x": -90, "y": 140, "angle": 45},
            {"key": "xshut5", "name": "Front_Bottom_Left", "signal": "tof.xshut5",
             "kind": "point", "x": -90, "y": 140, "angle": 45},
        ]

    def test_three_confirmed_frames(self):
        frame = {"tof.xshut6": 600, "tof.xshut5": 300}
        self.model._detect_weights(frame)
        self.model._detect_weights(frame)
        self.assertEqual(self.model.detected_weights, [])
        self.model._detect_weights(frame)
        self.assertEqual(len(self.model.detected_weights), 1)

    def test_invalid_or_small_gap_does_not_vote(self):
        self.model._detect_weights({"tof.xshut6": 600, "tof.xshut5": None})
        self.model._detect_weights({"tof.xshut6": 400, "tof.xshut5": 300})
        self.assertEqual(self.model.weight_votes.get("xshut6"), 0)
        self.assertEqual(self.model.detected_weights, [])

    def test_disabled_pair_does_not_create_weight_candidate(self):
        self.model.sensor_specs[0]["enabled"] = False
        for _ in range(3):
            self.model._detect_weights({"tof.xshut6": 600, "tof.xshut5": 300})
        self.assertEqual(self.model.detected_weights, [])
        self.assertEqual(self.model.weight_votes["xshut6"], 0)

    def test_reset_clears_local_candidates(self):
        frame = {"tof.xshut6": 600, "tof.xshut5": 300}
        for _ in range(3):
            self.model._detect_weights(frame)
        self.model.reset()
        self.assertEqual(self.model.detected_weights, [])


if __name__ == "__main__":
    unittest.main()
