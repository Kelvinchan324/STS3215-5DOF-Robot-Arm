import math
import unittest

from arm_helpers import angle_to_position, position_to_angle
from robot_math import (
    forward_kinematics,
    move_tip_forward,
    move_tip_height,
    tool_pitch_degrees,
)
import arm_settings as settings


def _cross_for_test(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


class ArmMathTests(unittest.TestCase):
    def test_joint_angle_round_trip(self):
        for name in settings.JOINT_NAMES:
            for angle in (-10.0, 0.0, 10.0):
                position = angle_to_position(name, angle)
                self.assertAlmostEqual(position_to_angle(name, position), angle, delta=0.1)

    def test_zero_angle_maps_to_configured_hardware_middle(self):
        for name in settings.JOINT_NAMES:
            self.assertEqual(angle_to_position(name, 0), settings.ZERO_TICKS[name])
            self.assertAlmostEqual(
                position_to_angle(name, settings.ZERO_TICKS[name]),
                0.0,
            )

    def test_servo_ids_two_three_and_four_use_normal_polarity(self):
        for name in settings.JOINT_NAMES:
            with self.subTest(joint=name):
                expected_direction = -1 if settings.SERVO_IDS[name] == 5 else 1
                self.assertEqual(settings.DIRECTIONS[name], expected_direction)
                positive_tick = settings.ZERO_TICKS[name] - 100
                self.assertEqual(
                    position_to_angle(name, positive_tick) > 0,
                    expected_direction == -1,
                )
                self.assertEqual(
                    angle_to_position(name, 10) < settings.ZERO_TICKS[name],
                    expected_direction == -1,
                )

    def test_gripper_angle_limits_are_minus_ninety_to_ninety(self):
        self.assertEqual(settings.ANGLE_LIMITS["gripper"], (-90.0, 90.0))
        self.assertEqual(angle_to_position("gripper", -90), 3072)
        self.assertEqual(angle_to_position("gripper", 90), 1024)

    def test_forward_kinematics_at_zero(self):
        angles = {name: 0.0 for name in settings.JOINT_NAMES}
        x, y, z = forward_kinematics(angles)
        self.assertAlmostEqual(x, -99.8543514)
        self.assertAlmostEqual(y, -97.3270974)
        self.assertAlmostEqual(z, 234.279446)

    def test_forward_kinematics_matches_cad_simulator_pose(self):
        angles = {name: 0.0 for name in settings.JOINT_NAMES}
        angles["shoulder"] = -7.5
        point = forward_kinematics(angles)
        base = settings.CAD_JOINT_POINTS_MM[0]
        cad_coordinates = (
            base[0] + point[0],
            base[1] + point[2],
            base[2] - point[1],
        )
        for actual, expected in zip(cad_coordinates, (-95.8, 219.7, 60.5)):
            self.assertAlmostEqual(actual, expected, places=1)

    def test_cad_dimensions_are_used_for_axis_geometry(self):
        self.assertEqual(len(settings.CAD_JOINT_POINTS_MM), 5)
        self.assertEqual(len(settings.CAD_JOINT_AXES), 5)
        self.assertAlmostEqual(
            math.dist(settings.CAD_JOINT_POINTS_MM[0], settings.CAD_JOINT_POINTS_MM[1]),
            93.4035,
            places=3,
        )
        self.assertAlmostEqual(
            math.dist(settings.CAD_JOINT_POINTS_MM[1], settings.CAD_JOINT_POINTS_MM[2]),
            120.4949,
            places=3,
        )
        self.assertAlmostEqual(
            math.dist(settings.CAD_JOINT_POINTS_MM[2], settings.CAD_JOINT_POINTS_MM[3]),
            154.6,
            places=2,
        )
        self.assertAlmostEqual(
            abs(sum(a * b for a, b in zip(settings.CAD_JOINT_AXES[0], settings.CAD_JOINT_AXES[1]))),
            0.0,
            places=7,
        )
        self.assertAlmostEqual(
            abs(sum(a * b for a, b in zip(settings.CAD_JOINT_AXES[3], settings.CAD_JOINT_AXES[4]))),
            0.0,
            places=7,
        )
        base_delta = tuple(
            end - start
            for start, end in zip(
                settings.CAD_JOINT_POINTS_MM[0],
                settings.CAD_JOINT_POINTS_MM[1],
            )
        )
        base_normal = _cross_for_test(
            settings.CAD_JOINT_AXES[0],
            settings.CAD_JOINT_AXES[1],
        )
        self.assertAlmostEqual(
            abs(sum(a * b for a, b in zip(base_delta, base_normal))),
            42.2073,
            places=3,
        )
        wrist_delta = tuple(
            end - start
            for start, end in zip(
                settings.CAD_JOINT_POINTS_MM[3],
                settings.CAD_JOINT_POINTS_MM[4],
            )
        )
        wrist_normal = _cross_for_test(
            settings.CAD_JOINT_AXES[3],
            settings.CAD_JOINT_AXES[4],
        )
        self.assertAlmostEqual(
            abs(sum(a * b for a, b in zip(wrist_delta, wrist_normal))),
            0.0,
            places=6,
        )

    def test_gripper_roll_does_not_move_wrist_axis_reference_point(self):
        angles = {name: 0.0 for name in settings.JOINT_NAMES}
        before = forward_kinematics(angles)
        angles["gripper"] = 75.0
        after = forward_kinematics(angles)
        for actual, expected in zip(after, before):
            self.assertAlmostEqual(actual, expected, places=8)

    def test_height_step_preserves_radial_reach_and_tool_pitch(self):
        angles = {
            "base_yaw": 0.0,
            "shoulder": 20.0,
            "elbow": 60.0,
            "wrist": -80.0,
            "gripper": 0.0,
        }
        before = forward_kinematics(angles)
        before_pitch = tool_pitch_degrees(angles)
        target = move_tip_height(angles, 2.0)
        after = forward_kinematics(target)
        self.assertAlmostEqual(math.hypot(before[0], before[1]), math.hypot(after[0], after[1]))
        self.assertAlmostEqual(after[2] - before[2], 2.0)
        self.assertAlmostEqual(tool_pitch_degrees(target), before_pitch, places=5)

    def test_forward_step_preserves_height_and_tool_pitch(self):
        angles = {
            "base_yaw": 0.0,
            "shoulder": 20.0,
            "elbow": 60.0,
            "wrist": -80.0,
            "gripper": 0.0,
        }
        before = forward_kinematics(angles)
        target = move_tip_forward(angles, 2.0)
        after = forward_kinematics(target)
        self.assertAlmostEqual(
            math.hypot(after[0], after[1]) - math.hypot(before[0], before[1]),
            2.0,
        )
        self.assertAlmostEqual(after[2], before[2])
        self.assertAlmostEqual(
            tool_pitch_degrees(target),
            tool_pitch_degrees(angles),
            places=5,
        )


if __name__ == "__main__":
    unittest.main()
