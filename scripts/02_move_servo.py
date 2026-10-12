#!/usr/bin/env python3
"""Move all five servos using raw position values."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arm_settings as settings
from arm_helpers import (
    connect,
    move_all_sync,
    wait_for_servos_or_estop,
)
from terminal_keys import RawTerminal


def parse_positions(raw_values):
    """Parse five raw position targets in configured joint order."""
    values = raw_values.split()
    if len(values) != len(settings.JOINT_NAMES):
        raise ValueError(f"Enter exactly {len(settings.JOINT_NAMES)} raw positions")

    positions = {}
    for name, value in zip(settings.JOINT_NAMES, values):
        try:
            position = int(value)
        except ValueError as error:
            raise ValueError(f"{name} raw position must be a whole number") from error
        if not 0 <= position <= 4095:
            raise ValueError(f"{name} raw position must be between 0 and 4095")
        positions[name] = position
    return positions


def main():
    joint_order = ", ".join(
        f"ID {settings.SERVO_IDS[name]} {name}" for name in settings.JOINT_NAMES
    )
    print(f"Enter raw positions in this servo-ID order: {joint_order}")
    raw_values = input("Five target raw positions (0-4095), separated by spaces: ")
    positions = parse_positions(raw_values)
    print("Target raw positions:")
    for name in settings.JOINT_NAMES:
        print(f"  ID {settings.SERVO_IDS[name]} ({name}): {positions[name]} ticks")
    input("Clear and support the arm, then press Enter to send the move...")
    port, packet_handler = connect()
    try:
        move_all_sync(packet_handler, positions)
        print("Synchronized move sent to all five servos. Press Space for E-stop.")
        with RawTerminal() as terminal:
            completed = wait_for_servos_or_estop(
                packet_handler,
                [settings.SERVO_IDS[name] for name in settings.JOINT_NAMES],
                terminal,
            )
        if completed:
            print("All five servos finished moving.")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
