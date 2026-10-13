import json
import runpy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import arm_settings as settings

sequence_module = runpy.run_path(
    "scripts/05_waypoint_sequence.py", run_name="waypoint_sequence_test"
)


class PoseSequenceTests(unittest.TestCase):
    def test_player_uses_configured_sequence_speed_and_acceleration(self):
        class FakePort:
            def closePort(self):
                pass

        class FakeTerminal:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                pass

            def read_key(self, _timeout):
                return None

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "speed_test.json"
            positions = {name: settings.ZERO_TICKS[name] for name in settings.JOINT_NAMES}
            path.write_text(
                json.dumps(
                    {
                        "type": "pose_sequence",
                        "name": "speed_test",
                        "input_type": "raw",
                        "points": [
                            {"input_values": ["2048"] * 5},
                            {"input_values": ["2048"] * 5},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            move = Mock()
            wait = Mock(return_value=True)
            overrides = {
                "connect": Mock(return_value=(FakePort(), object())),
                "enable_all_torque": Mock(),
                "move_all_sync": move,
                "wait_for_servos_or_estop": wait,
                "RawTerminal": FakeTerminal,
            }
            with patch.dict(
                sequence_module["play_sequence"].__globals__, overrides
            ):
                with patch("builtins.input", return_value=""):
                    sequence_module["play_sequence"](path)

        self.assertEqual(move.call_count, 2)
        self.assertTrue(
            all(
                call.kwargs["speed"] == settings.POSE_SEQUENCE_MOVE_SPEED
                and call.kwargs["acceleration"]
                == settings.POSE_SEQUENCE_MOVE_ACCELERATION
                for call in move.call_args_list
            )
        )

    def test_raw_point_parses_five_positions_in_configured_order(self):
        positions = sequence_module["parse_point"]("0 90 4095 2048 1234", "raw")
        self.assertEqual(list(positions), list(settings.JOINT_NAMES))
        self.assertEqual(list(positions.values()), [0, 90, 4095, 2048, 1234])

    def test_angle_point_converts_to_raw_positions(self):
        positions = sequence_module["parse_point"]("0 90 -90 90 90", "angles")
        expected = {
            name: sequence_module["angle_to_position"](name, angle)
            for name, angle in zip(settings.JOINT_NAMES, (0, 90, -90, 90, 90))
        }
        self.assertEqual(positions, expected)

    def test_point_validation_rejects_bad_count_range_and_angle(self):
        with self.assertRaisesRegex(ValueError, "exactly 5"):
            sequence_module["parse_point"]("0 0 0", "raw")
        with self.assertRaisesRegex(ValueError, "between 0 and 4095"):
            sequence_module["parse_point"]("0 0 -1 0 0", "raw")
        with self.assertRaisesRegex(ValueError, "outside the configured limits"):
            sequence_module["parse_point"]("0 0 0 0 91", "angles")

    def test_sequence_load_converts_minimal_angle_points_to_raw_positions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "demo_sequence.json"
            point_values = ["0", "10", "-10", "20", "0"]
            path.write_text(
                json.dumps(
                    {
                        "type": "pose_sequence",
                        "name": "demo_sequence",
                        "points": [
                            {"input_values": point_values},
                            {"input_values": point_values},
                        ],
                        "input_type": "angles",
                    }
                ),
                encoding="utf-8",
            )
            _data, points = sequence_module["load_sequence"](path)
        self.assertEqual(len(points), 2)
        self.assertEqual(list(points[0]), list(settings.JOINT_NAMES))
        self.assertEqual(
            points[0],
            sequence_module["parse_point_values"](point_values, "angles"),
        )

    def test_sequence_load_accepts_filename_when_saved_name_differs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test1.json"
            path.write_text(
                json.dumps(
                    {
                        "type": "pose_sequence",
                        "name": "gripper",
                        "input_type": "raw",
                        "points": [
                            {"input_values": ["2048"] * 5},
                            {"input_values": ["2048"] * 5},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            sequence, points = sequence_module["load_sequence"](path)
        self.assertEqual(sequence["name"], "gripper")
        self.assertEqual(len(points), 2)

    def test_sequence_load_still_accepts_legacy_raw_position_points(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.json"
            raw_positions = {
                name: settings.ZERO_TICKS[name] for name in settings.JOINT_NAMES
            }
            path.write_text(
                json.dumps(
                    {
                        "type": "pose_sequence",
                        "name": "legacy",
                        "input_type": "raw",
                        "points": [
                            {"positions_raw": raw_positions},
                            {"positions_raw": raw_positions},
                        ],
                    }
                ),
                encoding="utf-8",
            )
            _sequence, points = sequence_module["load_sequence"](path)
        self.assertEqual(points, [raw_positions, raw_positions])

    def test_entering_play_saves_sequence_and_returns_path_for_playback(self):
        with tempfile.TemporaryDirectory() as directory:
            create_globals = sequence_module["create_sequence"].__globals__
            answers = iter(
                (
                    "paper_demo",
                    "Example points",
                    "2",
                    "0 0 0 0 0",
                    "1 0 0 0 0",
                    "play",
                )
            )
            with patch.dict(create_globals, {"MOTION_FOLDER": Path(directory)}):
                with patch(
                    "builtins.input", side_effect=lambda _prompt: next(answers)
                ):
                    path = sequence_module["create_sequence"]()
            self.assertEqual(path, Path(directory) / "paper_demo.json")
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(len(data["points"]), 2)
            self.assertEqual(data["type"], "pose_sequence")
            self.assertEqual(data["name"], "paper_demo")
            self.assertEqual(
                data["joint_order"],
                [
                    {"servo_id": settings.SERVO_IDS[joint], "joint": joint}
                    for joint in settings.JOINT_NAMES
                ],
            )
            self.assertEqual(
                data["points"],
                [
                    {"input_values": ["0", "0", "0", "0", "0"]},
                    {"input_values": ["1", "0", "0", "0", "0"]},
                ],
            )
            self.assertTrue(
                all(set(point) == {"input_values"} for point in data["points"])
            )
            saved_text = path.read_text(encoding="utf-8")
            self.assertIn(
                '    {"input_values": ["0", "0", "0", "0", "0"]},',
                saved_text,
            )

    def test_regular_motion_picker_does_not_list_pose_sequences(self):
        motion_module = runpy.run_path(
            "scripts/04_timestamp_motion.py", run_name="motion_filter_test"
        )
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "demo_sequence.json").write_text(
                json.dumps({"type": "pose_sequence", "name": "demo_sequence"}),
                encoding="utf-8",
            )
            (folder / "demo_motion.json").write_text(
                json.dumps({"name": "demo_motion", "steps": []}),
                encoding="utf-8",
            )
            motion_module["find_motion_files"].__globals__["MOTION_FOLDER"] = folder
            self.assertEqual(
                [path.name for path in motion_module["find_motion_files"]()],
                ["demo_motion.json"],
            )


if __name__ == "__main__":
    unittest.main()
