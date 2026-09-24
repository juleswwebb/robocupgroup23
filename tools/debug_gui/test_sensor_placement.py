"""Only installed, placeable sensors appear in the arena layout."""

import unittest

from arena_view import ArenaModel, ArenaView
from wiring import HardwareMap


class FakeSettings:
    def __init__(self):
        self.values = {}

    def value(self, key, default=None, type=None):
        return self.values.get(key, default)


class PlacementTests(unittest.TestCase):
    def test_installed_sensor_names_and_heights_follow_wiring(self):
        holder = type("PlacementHolder", (), {})()
        holder.model = ArenaModel()
        holder.settings = FakeSettings()
        holder._placement_spec = ArenaView._placement_spec.__get__(holder)
        holder.hardware_map, warning = HardwareMap.load(HardwareMap.default_path())
        self.assertIsNone(warning)
        ArenaView._build_specs(holder)
        specs = {spec["key"]: spec for spec in holder.model.sensor_specs}
        self.assertEqual(len(specs), 12)
        self.assertEqual(specs["matrix"]["kind"], "matrix")
        for key in ("ultrasonic0", "ultrasonic1", "flow", "inductive"):
            self.assertIn(key, specs)
        self.assertEqual(specs["ultrasonic0"]["name"], "Ultrasonic_Right")
        self.assertEqual(specs["ultrasonic1"]["name"], "Ultrasonic_Left")
        self.assertEqual(specs["ultrasonic0"]["kind"], "ultrasonic")
        self.assertEqual(specs["matrix"]["name"], "8×8 ToF array")
        self.assertTrue(all("height_mm" in spec for spec in specs.values()))
        for key in ("ir0", "ir1", "ir2", "ir3", "colour", "imu", "encoder0", "encoder1"):
            self.assertNotIn(key, specs)

        holder.settings.values["arena/sensors/ultrasonic0/height_mm"] = 275
        ultrasonic = holder.hardware_map.device_for_signal("ultrasonic.0")
        ultrasonic.name = "Right sonar"
        holder.hardware_map.reindex()
        ArenaView._build_specs(holder)
        renamed = {spec["key"]: spec for spec in holder.model.sensor_specs}
        self.assertEqual(renamed["ultrasonic0"]["name"], "Right sonar")
        self.assertEqual(renamed["ultrasonic0"]["height_mm"], 275)


if __name__ == "__main__":
    unittest.main()
