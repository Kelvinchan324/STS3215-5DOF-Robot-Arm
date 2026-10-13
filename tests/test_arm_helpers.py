import runpy
import unittest

from arm_helpers import (
    disable_all_torque,
    enable_all_torque,
    move_all_sync,
)
import arm_settings as settings


class FakeSyncPacketHandler:
    def __init__(self):
        self.moves = []
        self.groupSyncWrite = self
        self.cleared = False

    def SyncWritePosEx(self, servo_id, position, speed, acceleration):
        self.moves.append((servo_id, position, speed, acceleration))
        return True

    def txPacket(self):
        return 0

    def clearParam(self):
        self.cleared = True

    @staticmethod
    def getTxRxResult(_result):
        return "communication error"


class FakeTorquePacketHandler:
    def __init__(self):
        self.torque_commands = []

    def write1ByteTxRx(self, servo_id, address, value):
        self.torque_commands.append((servo_id, address, value))
        return 0, 0

    @staticmethod
    def getTxRxResult(_result):
        return "communication error"

    @staticmethod
    def getRxPacketError(_error):
        return "servo error"


class MoveServoTests(unittest.TestCase):
    def test_read_positions_shows_raw_ticks_and_calculated_angles(self):
        module = runpy.run_path(
            "scripts/01_read_positions.py", run_name="read_positions_test"
        )
        raw_positions = {
            name: settings.ZERO_TICKS[name] for name in settings.JOINT_NAMES
        }
        formatted = module["format_positions"](raw_positions)
        self.assertEqual(len(formatted), len(settings.JOINT_NAMES))
        for name, servo_id, raw, angle in formatted:
            self.assertEqual(servo_id, settings.SERVO_IDS[name])
            self.assertEqual(raw, settings.ZERO_TICKS[name])
            self.assertAlmostEqual(angle, 0.0)

    def test_script_02_parses_five_raw_positions_in_joint_order(self):
        module = runpy.run_path(
            "scripts/02_move_servo.py", run_name="move_servo_test"
        )
        positions = module["parse_positions"]("0 90 4095 2048 1234")
        self.assertEqual(
            list(positions.keys()),
            list(settings.JOINT_NAMES),
        )
        self.assertEqual(list(positions.values()), [0, 90, 4095, 2048, 1234])

    def test_script_02_rejects_wrong_count_and_invalid_raw_positions(self):
        module = runpy.run_path(
            "scripts/02_move_servo.py", run_name="move_servo_test"
        )
        with self.assertRaisesRegex(ValueError, "exactly 5 raw positions"):
            module["parse_positions"]("0 0 0 0")
        with self.assertRaisesRegex(ValueError, "between 0 and 4095"):
            module["parse_positions"]("0 0 -90 0 0")
        with self.assertRaisesRegex(ValueError, "whole number"):
            module["parse_positions"]("0 0 0.5 0 0")

    def test_angle_move_script_converts_five_angles_in_joint_order(self):
        module = runpy.run_path(
            "scripts/03_move_joint_angle.py", run_name="move_joint_angle_test"
        )
        positions = module["parse_angles"]("0 10 -10 20 0")
        expected = {
            name: module["angle_to_position"](name, angle)
            for name, angle in zip(settings.JOINT_NAMES, (0, 10, -10, 20, 0))
        }
        self.assertEqual(positions, expected)

    def test_angle_move_script_shows_configured_minimum_and_maximum(self):
        module = runpy.run_path(
            "scripts/03_move_joint_angle.py", run_name="move_joint_angle_test"
        )
        prompt = module["format_joint_limits"]()
        for name in settings.JOINT_NAMES:
            minimum, maximum = settings.ANGLE_LIMITS[name]
            expected = (
                f"ID {settings.SERVO_IDS[name]} {name} "
                f"[{minimum:g} to {maximum:g} deg]"
            )
            self.assertIn(expected, prompt)

    def test_angle_move_script_rejects_wrong_count_and_out_of_range_angles(self):
        module = runpy.run_path(
            "scripts/03_move_joint_angle.py", run_name="move_joint_angle_test"
        )
        with self.assertRaisesRegex(ValueError, "exactly 5"):
            module["parse_angles"]("0 0 0 0")
        with self.assertRaisesRegex(ValueError, "outside the configured limits"):
            module["parse_angles"]("0 0 0 0 91")
        with self.assertRaisesRegex(ValueError, "must be finite"):
            module["parse_angles"]("0 0 0 0 nan")


    def test_synchronized_move_uses_all_configured_servos_and_clears_parameters(self):
        packet_handler = FakeSyncPacketHandler()
        targets = {name: 2048 for name in settings.JOINT_NAMES}
        move_all_sync(packet_handler, targets, speed=800, acceleration=50)
        self.assertEqual(
            [move[0] for move in packet_handler.moves],
            [settings.SERVO_IDS[name] for name in settings.JOINT_NAMES],
        )
        self.assertTrue(all(move[2:] == (800, 50) for move in packet_handler.moves))
        self.assertTrue(packet_handler.cleared)

    def test_emergency_stop_disables_torque_for_every_servo(self):
        packet_handler = FakeTorquePacketHandler()
        disable_all_torque(packet_handler)
        self.assertEqual(
            packet_handler.torque_commands,
            [
                (servo_id, 40, 0)
                for servo_id in settings.SERVO_IDS.values()
            ],
        )

    def test_enable_torque_enables_every_servo(self):
        packet_handler = FakeTorquePacketHandler()
        enable_all_torque(packet_handler)
        self.assertEqual(
            packet_handler.torque_commands,
            [
                (servo_id, 40, 1)
                for servo_id in settings.SERVO_IDS.values()
            ],
        )


if __name__ == "__main__":
    unittest.main()
