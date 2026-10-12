import json
import math
from pathlib import Path
import unittest

from cad_forward_kinematics import position_from_raw


class CADForwardTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((Path(__file__).resolve().parents[1] / "kinematics_config.json").read_text())
        self.raw = {j["servo_id"]: j["home_position"] for j in self.config["joints"]}

    def test_reference_home_and_coordinate_frames(self):
        result = position_from_raw(self.raw, self.config)
        self.assertEqual(result["point"], "wrist_reference")
        for actual, expected in zip(result["xyz"], (-99.8543514, -97.3270974, 234.279446)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
                position_from_raw(self.raw, self.config, frame="cad")["xyz"],
                self.config["joints"][-1]["point_mm"]):
            self.assertAlmostEqual(actual, expected)

    def test_multiple_joint_rotations_match_existing_engine(self):
        import arm_settings
        from robot_math import forward_kinematics
        self.raw.update({1: 2100, 2: 2800, 3: 1600, 4: 2300, 5: 2400})
        result = position_from_raw(self.raw, self.config)
        angles = dict(zip(arm_settings.JOINT_NAMES,
                          map(math.degrees, result["angles_rad"])))
        for actual, expected in zip(result["xyz"], forward_kinematics(angles)):
            self.assertAlmostEqual(actual, expected)

    def test_base_quarter_turn_rotates_a_measured_tcp(self):
        pivot = self.config["joints"][0]["point_mm"]
        tcp = (pivot[0] + 100, pivot[1], pivot[2])
        self.raw[1] += 1024
        result = position_from_raw(self.raw, self.config, tcp)
        self.assertEqual(result["point"], "gripper_tcp")
        for actual, expected in zip(result["xyz"], (0, 100, 0)):
            self.assertAlmostEqual(actual, expected)

    def test_roll_rotates_off_axis_tcp_but_preserves_wrist(self):
        wrist = position_from_raw(self.raw, self.config)["xyz"]
        tcp = tuple(v + d for v, d in zip(self.config["joints"][-1]["point_mm"], (0, 10, 0)))
        before = position_from_raw(self.raw, self.config, tcp)["xyz"]
        self.raw[5] += 512
        after = position_from_raw(self.raw, self.config, tcp)["xyz"]
        self.assertGreater(math.dist(before, after), 1)
        for actual, expected in zip(position_from_raw(self.raw, self.config)["xyz"], wrist):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(math.dist(after, wrist), 10)

    def test_invalid_encoder_and_tcp_are_rejected(self):
        self.raw[2] = -1
        with self.assertRaises(ValueError):
            position_from_raw(self.raw, self.config)
        self.raw[2] = 3164
        with self.assertRaises(ValueError):
            position_from_raw(self.raw, self.config, (float("nan"), 0, 0))
