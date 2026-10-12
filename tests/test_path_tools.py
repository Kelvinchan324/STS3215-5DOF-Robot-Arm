import math
import runpy
import unittest

from robot_math import forward_kinematics, inverse_kinematics, tool_pitch_degrees
import arm_settings as settings

record_module = runpy.run_path(
    "scripts/09_record_path.py", run_name="record_path_test"
)
play_module = runpy.run_path("scripts/10_play_path.py", run_name="play_path_test")


class CartesianPathTests(unittest.TestCase):
    def setUp(self):
        self.angles = {
            "base_yaw": 15.0,
            "shoulder": 20.0,
            "elbow": 60.0,
            "wrist": -80.0,
            "gripper": 0.0,
        }

    def test_record_waypoint_contains_fk_position_and_raw_pose(self):
        raw_positions = {name: 2048 for name in settings.JOINT_NAMES}
        waypoint = record_module["make_waypoint"](1, raw_positions)
        xyz = forward_kinematics(waypoint["angles_degrees"])
        self.assertEqual(waypoint["positions_raw"], raw_positions)
        for actual, saved in zip(xyz, waypoint["position_mm"].values()):
            self.assertAlmostEqual(actual, saved)

    def test_inverse_kinematics_reconstructs_fk_target(self):
        xyz = forward_kinematics(self.angles)
        pitch = tool_pitch_degrees(self.angles)
        result = inverse_kinematics(*xyz, pitch, self.angles)
        reconstructed = forward_kinematics(result)
        for actual, expected in zip(reconstructed, xyz):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_interpolated_segment_reaches_cartesian_end(self):
        start_xyz = forward_kinematics(self.angles)
        end_angles = dict(self.angles)
        end_angles["shoulder"] += 2.0
        end_xyz = forward_kinematics(end_angles)

        def waypoint(angles, xyz):
            return {
                "angles_degrees": angles,
                "position_mm": dict(zip(("x", "y", "z"), xyz)),
                "tool_pitch_degrees": tool_pitch_degrees(angles),
            }

        targets = list(
            play_module["interpolate_segment"](
                waypoint(self.angles, start_xyz),
                waypoint(end_angles, end_xyz),
                self.angles,
            )
        )
        result_xyz = forward_kinematics(targets[-1])
        self.assertGreater(len(targets), 1)
        for actual, expected in zip(result_xyz, end_xyz):
            self.assertAlmostEqual(actual, expected, places=5)


if __name__ == "__main__":
    unittest.main()
