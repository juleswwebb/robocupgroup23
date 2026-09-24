"""Two-run forward encoder calibration and odometry sign regression."""

import unittest

from arena_view import ArenaModel, ArenaView, DEFAULT_LEFT_MM_PER_COUNT, DEFAULT_RIGHT_MM_PER_COUNT


class Settings:
    def __init__(self, values=None):
        self.values = dict(values or {})

    def value(self, key, default=None, type=None):
        return self.values.get(key, default)

    def setValue(self, key, value):
        self.values[key] = value


def load_calibration(settings):
    holder = type("CalibrationHolder", (), {})()
    holder.model = ArenaModel()
    holder.settings = settings
    ArenaView._load_calibration(holder)
    return holder.model


class EncoderCalibrationTests(unittest.TestCase):
    def test_fresh_settings_use_measured_defaults(self):
        model = load_calibration(Settings())
        self.assertAlmostEqual(model.left_mm_per_count, DEFAULT_LEFT_MM_PER_COUNT)
        self.assertAlmostEqual(model.right_mm_per_count, DEFAULT_RIGHT_MM_PER_COUNT)
        self.assertTrue(model.invert_right)

    def test_untouched_legacy_defaults_are_upgraded_once(self):
        settings = Settings({"arena/left_scale": 0.095, "arena/right_scale": 0.095,
                             "arena/invert_left": False, "arena/invert_right": False})
        model = load_calibration(settings)
        self.assertAlmostEqual(model.left_mm_per_count, DEFAULT_LEFT_MM_PER_COUNT)
        self.assertAlmostEqual(model.right_mm_per_count, DEFAULT_RIGHT_MM_PER_COUNT)
        self.assertTrue(model.invert_right)
        self.assertEqual(settings.values["arena/encoder_calibration_version"], 1)
        settings.setValue("arena/left_scale", 0.095)
        self.assertEqual(load_calibration(settings).left_mm_per_count, 0.095)

    def test_custom_saved_calibration_is_preserved(self):
        settings = Settings({"arena/left_scale": 0.081, "arena/right_scale": 0.083,
                             "arena/invert_left": True, "arena/invert_right": False})
        model = load_calibration(settings)
        self.assertEqual(model.left_mm_per_count, 0.081)
        self.assertEqual(model.right_mm_per_count, 0.083)
        self.assertTrue(model.invert_left)
        self.assertFalse(model.invert_right)

    def test_two_run_defaults_and_forward_polarity(self):
        model = ArenaModel()
        self.assertAlmostEqual(model.left_mm_per_count, 3635 / 41153)
        self.assertAlmostEqual(model.right_mm_per_count, 3635 / 42224)
        self.assertFalse(model.invert_left)
        self.assertTrue(model.invert_right)

        model._integrate_pose({"encoder.0": 0, "encoder.1": 0})
        model._integrate_pose({"encoder.0": 41153, "encoder.1": -42224})
        self.assertAlmostEqual(model.distance_mm, 3635, places=2)
        self.assertAlmostEqual(model.y, 3635, places=2)


if __name__ == "__main__":
    unittest.main()
