#!/usr/bin/env python3
"""Read and display the current positions of the five STS3215 servos."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arm_settings as settings
from arm_helpers import (
    connect,
    disable_all_torque,
    position_to_angle,
    read_all,
)


def format_positions(raw_positions):
    """Format each servo's raw position and angle relative to its configured zero."""
    return [
        (
            name,
            settings.SERVO_IDS[name],
            raw_positions[name],
            position_to_angle(name, raw_positions[name]),
        )
        for name in settings.JOINT_NAMES
    ]


def main():
    print(f"Opening {settings.SERIAL_PORT} at {settings.BAUD_RATE} baud...")
    port = None
    try:
        port, packet_handler = connect()
        disable_all_torque(packet_handler)
        print("Torque disabled on all configured servos.")
        raw_positions = read_all(packet_handler)
        print("Current servo positions and angles:")
        for name, servo_id, position, angle in format_positions(raw_positions):
            print(
                f"  ID {servo_id} {name:12s} raw: {position:4d} ticks, "
                f"angle: {angle:7.2f} deg"
            )
    except Exception as error:
        print(f"\nCould not read the servos: {error}", file=sys.stderr)
        print(
            "Check that the website is disconnected, the selected baud rate "
            "matches its baud setting, the USB port is correct, and the servo "
            "power is on. If some IDs respond but not others, check the IDs "
            "in arm_settings.py. Torque is disabled before reading, so support "
            "the arm to prevent gravity from moving it.",
            file=sys.stderr,
        )
        raise SystemExit(1) from error
    finally:
        if port is not None:
            port.closePort()


if __name__ == "__main__":
    main()
