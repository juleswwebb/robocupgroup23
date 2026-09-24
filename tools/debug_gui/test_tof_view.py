"""Checks that the new 8x8 display filters don't invent missing returns."""

import unittest
from collections import deque

import numpy as np

from tof_view import frame_from_telemetry, stable_detail, suppress_isolated_spikes


class TofViewProcessingTests(unittest.TestCase):
    def test_telemetry_preserves_absent_zones(self):
        frame = frame_from_telemetry({"tof.array.r2c3": 412, "tof.array.r2c4": None})
        self.assertEqual(frame[2, 3], 412)
        self.assertTrue(np.isnan(frame[2, 4]))

    def test_spatial_filter_replaces_only_an_isolated_spike(self):
        frame = np.full((8, 8), 500.0)
        frame[3, 3] = 2500
        frame[0, 0] = np.nan
        result = suppress_isolated_spikes(frame)
        self.assertEqual(result[3, 3], 500)
        self.assertTrue(np.isnan(result[0, 0]))
        self.assertEqual(frame[3, 3], 2500)  # raw frame is never modified

    def test_stable_detail_accepts_near_object_and_absent_return(self):
        history = deque(maxlen=5)
        far = np.full((8, 8), 1000.0)
        previous = stable_detail(history, None, far)
        near = far.copy()
        near[4, 4] = 300
        near[3, 3] = np.nan
        result = stable_detail(history, previous, near)
        self.assertEqual(result[4, 4], 300)
        self.assertTrue(np.isnan(result[3, 3]))


if __name__ == "__main__":
    unittest.main()
