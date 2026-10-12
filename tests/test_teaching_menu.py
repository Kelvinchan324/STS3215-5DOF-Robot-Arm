import subprocess
import sys
import unittest
from pathlib import Path

import run_robot_arm


class TeachingMenuTests(unittest.TestCase):
    def test_lesson_scripts_are_listed_and_exist(self):
        script_names = [
            script_name
            for _lesson_name, _objective, activities in run_robot_arm.LESSONS
            for _activity_name, script_name in activities
        ]
        self.assertEqual(
            set(script_names),
            {
                "01_read_positions.py",
                "02_move_servo.py",
                "03_move_joint_angle.py",
                "04_forward_kinematics.py",
                "05_timestamp_motion.py",
                "06_waypoint_sequence.py",
                "07_keyboard_control.py",
                "08_robot_arm_gui.py",
                "09_record_path.py",
                "10_play_path.py",
            },
        )
        self.assertEqual(len(run_robot_arm.LESSONS), 6)
        self.assertTrue(
            all(
                (run_robot_arm.SCRIPTS_FOLDER / script_name).is_file()
                for script_name in script_names
            )
        )

    def test_practice_parts_offer_timestamp_and_waypoint_tools(self):
        placement = run_robot_arm.LESSONS[3]
        attack = run_robot_arm.LESSONS[4]
        self.assertEqual(
            [script for _label, script in placement[2]],
            ["05_timestamp_motion.py", "06_waypoint_sequence.py"],
        )
        self.assertEqual(
            [script for _label, script in attack[2]],
            ["05_timestamp_motion.py", "06_waypoint_sequence.py"],
        )
        self.assertIn("block_placement_", placement[1])
        self.assertIn("block_attack_", attack[1])

    def test_part_two_names_script_04_as_the_kinematics_visualizer(self):
        part_two = run_robot_arm.LESSONS[1]
        self.assertEqual(part_two[2][2][0], "Visualize robot-arm kinematics")
        self.assertEqual(part_two[2][2][1], "04_forward_kinematics.py")

    def test_menu_returns_to_exit_on_q(self):
        result = subprocess.run(
            [sys.executable, str(Path(run_robot_arm.__file__))],
            input="q\n",
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Goodbye.", result.stdout)
        self.assertIn("Part 6: Robot arm control coding", result.stdout)


if __name__ == "__main__":
    unittest.main()
