"""Only installed, placeable sensors appear in the arena layout."""

import unittest

from arena_view import ArenaModel, ArenaView
from wiring import HardwareMap, PORTS_BY_ID, default_devices


class FakeSettings:
    def __init__(self):
        self.values = {}

    def value(self, key, default=None, type=None):
        return self.values.get(key, default)

    def setValue(self, key, value):
        self.values[key] = value


class PlacementTests(unittest.TestCase):
    def test_installed_sensor_names_and_heights_follow_wiring(self):
        holder = type("PlacementHolder", (), {})()
        holder.model = ArenaModel()
        holder.settings = FakeSettings()
        holder._placement_spec = ArenaView._placement_spec.__get__(holder)
        holder._migrate_range_sensor_defaults = ArenaView._migrate_range_sensor_defaults.__get__(holder)
        holder.hardware_map, warning = HardwareMap.load(HardwareMap.default_path())
        self.assertIsNone(warning)
        ArenaView._build_specs(holder)
        specs = {spec["key"]: spec for spec in holder.model.sensor_specs}
        point_count = sum(d.port.startswith("xshut") and d.kind in ("vl53l0x", "vl53l1x")
                          for d in holder.hardware_map.devices)
        active_ultrasonics = sum(holder.hardware_map.device_for_signal(f"ultrasonic.{n}") is not None
                                 for n in (0, 1))
        expected_count = point_count + active_ultrasonics
        expected_count += holder.hardware_map.device_for_signal("tof.8x8") is not None
        expected_count += holder.hardware_map.device_for_signal("inductive.detected") is not None
        expected_count += holder.hardware_map.device_for_signal("flow.dx") is not None
        self.assertEqual(len(specs), expected_count)
        self.assertEqual(specs["matrix"]["kind"], "matrix")
        for key in ("ultrasonic0", "ultrasonic1", "inductive"):
            self.assertIn(key, specs)
        self.assertEqual(specs["xshut6"]["angle"], 45)
        self.assertEqual(specs["xshut5"]["angle"], 45)
        self.assertEqual(specs["xshut3"]["angle"], -45)
        self.assertEqual(specs["xshut4"]["angle"], -45)
        self.assertEqual((specs["xshut7"]["name"], specs["xshut7"]["x"],
                          specs["xshut7"]["y"], specs["xshut7"]["angle"]),
                         ("Top_Right", 90, 140, 0))
        self.assertEqual((specs["xshut8"]["name"], specs["xshut8"]["x"],
                          specs["xshut8"]["y"], specs["xshut8"]["angle"]),
                         ("Top_Left", -90, 140, 0))
        self.assertNotIn("xshut0", specs)  # removed top-left L0X
        self.assertNotIn("xshut1", specs)  # removed top-right L0X
        self.assertTrue(specs["xshut7"]["enabled"])
        self.assertTrue(specs["xshut8"]["enabled"])
        self.assertIn("IO8", PORTS_BY_ID["xshut3"].label)
        self.assertIn("IO9", PORTS_BY_ID["xshut7"].label)
        self.assertIn("IO10", PORTS_BY_ID["xshut8"].label)
        self.assertEqual(specs["ultrasonic0"]["name"], "Ultrasonic_Right")
        self.assertEqual(specs["ultrasonic1"]["name"], "Ultrasonic_Left")
        self.assertEqual(specs["ultrasonic0"]["kind"], "ultrasonic")
        self.assertEqual((specs["ultrasonic0"]["x"], specs["ultrasonic0"]["y"],
                          specs["ultrasonic0"]["angle"]), (110, 0, 90))
        self.assertEqual((specs["ultrasonic1"]["x"], specs["ultrasonic1"]["y"],
                          specs["ultrasonic1"]["angle"]), (-110, 0, -90))
        self.assertEqual(specs["matrix"]["name"], "8×8 ToF array")
        self.assertTrue(all("height_mm" in spec for spec in specs.values()))
        for key in ("ir0", "ir1", "ir2", "ir3", "colour", "imu", "encoder0", "encoder1"):
            self.assertNotIn(key, specs)
        if holder.hardware_map.device_for_signal("flow.dx") is not None:
            self.assertIn("flow", specs)
        else:
            self.assertNotIn("flow", specs)

        holder.settings.values["arena/sensors/ultrasonic0/height_mm"] = 275
        ultrasonic = holder.hardware_map.device_for_signal("ultrasonic.0")
        ultrasonic.name = "Right sonar"
        holder.hardware_map.reindex()
        ArenaView._build_specs(holder)
        renamed = {spec["key"]: spec for spec in holder.model.sensor_specs}
        self.assertEqual(renamed["ultrasonic0"]["name"], "Right sonar")
        self.assertEqual(renamed["ultrasonic0"]["height_mm"], 275)

    def test_saved_placement_and_disable_survive_wiring_rename(self):
        holder = type("PlacementHolder", (), {})()
        holder.model = ArenaModel()
        holder.settings = FakeSettings()
        holder.settings.values.update({
            "arena/sensors/xshut7/x": 125,
            "arena/sensors/xshut7/angle": 7,
            "arena/sensors/xshut8/enabled": False,
        })
        holder._placement_spec = ArenaView._placement_spec.__get__(holder)
        holder._migrate_range_sensor_defaults = ArenaView._migrate_range_sensor_defaults.__get__(holder)
        holder.hardware_map, _ = HardwareMap.load(HardwareMap.default_path())
        holder.hardware_map.device_for_signal("tof.xshut7").name = "Renamed right sensor"
        ArenaView._build_specs(holder)
        specs = {spec["key"]: spec for spec in holder.model.sensor_specs}
        self.assertEqual((specs["xshut7"]["x"], specs["xshut7"]["angle"]), (125, 7))
        self.assertFalse(specs["xshut8"]["enabled"])
        self.assertEqual((specs["xshut3"]["x"], specs["xshut3"]["angle"]), (90, -45))

    def test_fallback_wiring_matches_six_installed_l1_and_two_sonars(self):
        devices = {device.port: device for device in default_devices()}
        self.assertEqual({port for port in devices if port.startswith("xshut")},
                         {f"xshut{n}" for n in range(3, 9)})
        self.assertTrue(all(devices[f"xshut{n}"].kind == "vl53l1x"
                            for n in range(3, 9)))
        self.assertEqual(devices["xshut7"].name, "Top_Right")
        self.assertEqual(devices["xshut8"].name, "Top_Left")
        self.assertIn("ultrasonic_30_31", devices)
        self.assertIn("ultrasonic_32_33", devices)

    def test_old_automatic_sensor_angles_migrate_but_custom_positions_survive(self):
        holder = type("PlacementHolder", (), {})()
        holder.model = ArenaModel()
        holder.settings = FakeSettings()
        holder.settings.values.update({
            "arena/sensors/xshut0/angle": -45,
            "arena/sensors/ultrasonic0/x": -110,
            "arena/sensors/ultrasonic0/y": 120,
            "arena/sensors/ultrasonic0/angle": 0,
            "arena/sensors/xshut1/angle": 12,
        })
        holder._placement_spec = ArenaView._placement_spec.__get__(holder)
        holder._migrate_range_sensor_defaults = ArenaView._migrate_range_sensor_defaults.__get__(holder)
        holder.hardware_map, warning = HardwareMap.load(HardwareMap.default_path())
        self.assertIsNone(warning)
        ArenaView._build_specs(holder)
        specs = {spec["key"]: spec for spec in holder.model.sensor_specs}
        self.assertNotIn("xshut0", specs)
        self.assertEqual(specs["ultrasonic0"]["x"], 110)
        self.assertEqual(specs["ultrasonic0"]["y"], 0)
        self.assertEqual(specs["ultrasonic0"]["angle"], 90)
        self.assertNotIn("xshut1", specs)
        self.assertEqual(holder.settings.values["arena/sensors/xshut0/angle"], 0)
        self.assertEqual(holder.settings.values["arena/sensors/xshut1/angle"], 12)
        self.assertEqual(holder.settings.values["arena/range_sensor_defaults_version"], 2)


if __name__ == "__main__":
    unittest.main()
