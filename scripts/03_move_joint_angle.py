#!/usr/bin/env python3
"""Move all five joints using target angles in degrees."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arm_settings as settings
from arm_helpers import (
    angle_to_position,
    connect,
    move_all_sync,
    wait_for_servos_or_estop,
)
from terminal_keys import RawTerminal


def parse_angles(raw_values):
    """Convert five target angles to raw positions in configured joint order."""
    values = raw_values.split()
    if len(values) != len(settings.JOINT_NAMES):
        raise ValueError(f"Enter exactly {len(settings.JOINT_NAMES)} angles")

    positions = {}
    for name, value in zip(settings.JOINT_NAMES, values):
        try:
            angle = float(value)
        except ValueError as error:
            raise ValueError(f"{name} angle must be a number") from error
        if not math.isfinite(angle):
            raise ValueError(f"{name} angle must be finite")
        positions[name] = angle_to_position(name, angle)
    return positions


def format_joint_limits():
    """Show configured input limits alongside each servo ID and joint."""
    return ", ".join(
        f"ID {settings.SERVO_IDS[name]} {name} "
        f"[{settings.ANGLE_LIMITS[name][0]:g} to "
        f"{settings.ANGLE_LIMITS[name][1]:g} deg]"
        for name in settings.JOINT_NAMES
    )


def main():
    print("Target angles in servo-ID order (minimum to maximum):")
    print(f"  {format_joint_limits()}")
    raw_values = input("Five target angles, separated by spaces: ")
    positions = parse_angles(raw_values)
    print("Converted target positions:")
    for name in settings.JOINT_NAMES:
        print(
            f"  ID {settings.SERVO_IDS[name]} ({name}): "
            f"{positions[name]} ticks"
        )
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
