#!/usr/bin/env python3
"""Jog continuously; Space releases torque on all servos."""

import sys
import time
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_FOLDER))

import arm_settings as settings
from arm_helpers import (
    angle_to_position,
    angles_to_positions,
    connect,
    disable_all_torque,
    move_positions_sync,
    positions_to_angles,
    read_all,
)
from robot_math import move_tip_forward, move_tip_height
from terminal_keys import RawTerminal

JOG_KEYS = {"a", "d", "w", "s", "f", "b"}
JOINTS = tuple(settings.JOINT_NAMES)


def advance_jog(angles, key, elapsed):
    """Calculate the next small target step for the active jog direction."""
    if key in {"w", "s", "f", "b"} and not settings.CAD_ZERO_ALIGNMENT_CONFIRMED:
        raise ValueError(
            "Cartesian jogging is disabled until arm_settings.py confirms that "
            "the Pi angle zeroes match the CAD home pose"
        )
    if key == "a":
        result = dict(angles)
        result["base_yaw"] += settings.KEYBOARD_YAW_SPEED_DEG_PER_SECOND * elapsed
        return result, ("base_yaw",)
    if key == "d":
        result = dict(angles)
        result["base_yaw"] -= settings.KEYBOARD_YAW_SPEED_DEG_PER_SECOND * elapsed
        return result, ("base_yaw",)
    if key == "w":
        return (
            move_tip_height(
                angles, settings.KEYBOARD_HEIGHT_SPEED_MM_PER_SECOND * elapsed
            ),
            JOINTS[:-1],
        )
    if key == "s":
        return (
            move_tip_height(
                angles, -settings.KEYBOARD_HEIGHT_SPEED_MM_PER_SECOND * elapsed
            ),
            JOINTS[:-1],
        )
    if key == "f":
        return (
            move_tip_forward(
                angles, settings.KEYBOARD_FORWARD_SPEED_MM_PER_SECOND * elapsed
            ),
            JOINTS[:-1],
        )
    if key == "b":
        return (
            move_tip_forward(
                angles, -settings.KEYBOARD_FORWARD_SPEED_MM_PER_SECOND * elapsed
            ),
            JOINTS[:-1],
        )
    raise ValueError(f"Unknown jog key: {key}")


def main():
    if not sys.stdin.isatty():
        raise RuntimeError(
            "Keyboard control needs a terminal. Run this script in a terminal window."
        )

    print("Connecting to the robot...")
    port, packet_handler = connect()
    try:
        angles = positions_to_angles(read_all(packet_handler))
        active_key = None
        previous_update = time.monotonic()
        print("Connected. Tap a direction to start continuous movement.")
        print(
            "a/d: yaw | w/s: height | f/b: forward/back | "
            "o: open | p: close | Space: EMERGENCY STOP (release torque) | q: quit"
        )
        if not settings.CAD_ZERO_ALIGNMENT_CONFIRMED:
            print(
                "Cartesian jog is disabled: verify Pi angle zeroes against the "
                "CAD home pose first."
            )
        print(
            f"Jog rates: yaw {settings.KEYBOARD_YAW_SPEED_DEG_PER_SECOND:g} deg/s, "
            f"height {settings.KEYBOARD_HEIGHT_SPEED_MM_PER_SECOND:g} mm/s, "
            f"forward/back {settings.KEYBOARD_FORWARD_SPEED_MM_PER_SECOND:g} mm/s"
        )

        with RawTerminal() as terminal:
            while True:
                now = time.monotonic()
                wait = max(
                    0.0,
                    settings.KEYBOARD_CONTROL_INTERVAL_SECONDS
                    - (now - previous_update),
                )
                key = terminal.read_key(wait)

                if key == "q":
                    break
                if key == " ":
                    active_key = None
                    print("\nEMERGENCY STOP: releasing torque on all servos.", flush=True)
                    disable_all_torque(packet_handler)
                    break
                if key in JOG_KEYS:
                    if key != active_key:
                        active_key = key
                        print(
                            f"\nJogging {key}; press Space to stop.",
                            flush=True,
                        )
                    previous_update = time.monotonic()
                    continue
                if key in ("o", "p"):
                    active_key = None
                    minimum, maximum = settings.ANGLE_LIMITS["gripper"]
                    angle = minimum if key == "o" else maximum
                    target = {"gripper": angle_to_position("gripper", angle)}
                    move_positions_sync(
                        packet_handler,
                        target,
                        ("gripper",),
                        speed=settings.KEYBOARD_MOVE_SPEED,
                        acceleration=settings.KEYBOARD_MOVE_ACCELERATION,
                    )
                    print(
                        "\nGripper open command sent."
                        if key == "o"
                        else "\nGripper close command sent.",
                        flush=True,
                    )
                    previous_update = time.monotonic()
                    continue

                if active_key is None:
                    continue

                now = time.monotonic()
                elapsed = now - previous_update
                if elapsed < settings.KEYBOARD_CONTROL_INTERVAL_SECONDS:
                    continue
                elapsed = min(elapsed, settings.KEYBOARD_CONTROL_INTERVAL_SECONDS * 2)
                try:
                    next_angles, moving_joints = advance_jog(
                        angles, active_key, elapsed
                    )
                    targets = angles_to_positions(next_angles)
                    move_positions_sync(
                        packet_handler,
                        targets,
                        moving_joints,
                        speed=settings.KEYBOARD_MOVE_SPEED,
                        acceleration=settings.KEYBOARD_MOVE_ACCELERATION,
                    )
                    angles = next_angles
                    previous_update = now
                except ValueError as error:
                    print(f"\nJog stopped: {error}", flush=True)
                    active_key = None
                    previous_update = now
    finally:
        port.closePort()
    print("\nKeyboard control stopped.")


if __name__ == "__main__":
    main()
