#!/usr/bin/env python3
"""Record manually posed arm waypoints and their forward-kinematics positions."""

import json
import re
import sys
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_FOLDER))

import arm_settings as settings
from arm_helpers import connect, disable_all_torque, positions_to_angles, read_all
from robot_math import forward_kinematics, tool_pitch_degrees

MOTION_FOLDER = PROJECT_FOLDER / "motions"
NAME_PATTERN = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,47}$")


def make_waypoint(number, raw_positions):
    angles = positions_to_angles(raw_positions)
    x, y, z = forward_kinematics(angles)
    return {
        "number": number,
        "positions_raw": raw_positions,
        "angles_degrees": angles,
        "position_mm": {"x": x, "y": y, "z": z},
        "tool_pitch_degrees": tool_pitch_degrees(angles),
    }


def main():
    if not settings.CAD_ZERO_ALIGNMENT_CONFIRMED:
        raise RuntimeError(
            "Cartesian path recording is disabled until arm_settings.py confirms "
            "that the Pi angle zeroes match the CAD home pose"
        )
    print(
        f"Connecting to {settings.SERIAL_PORT} at {settings.BAUD_RATE} baud. "
        "The arm may move when torque is released."
    )
    input("Support the arm and press Enter to release torque...")

    port, packet_handler = connect()
    try:
        disable_all_torque(packet_handler)
        print("Torque released on all configured servos.")

        name = input("Path name (letters, numbers, _ or -): ").strip()
        if not NAME_PATTERN.fullmatch(name):
            raise ValueError("Use 1-48 letters, numbers, underscores, or hyphens")
        description = input("Path description: ").strip()
        path = MOTION_FOLDER / f"{name}.json"
        if path.exists():
            raise FileExistsError(f"{path} already exists; choose a new name")

        print(
            "\nManually pose the arm. Press Enter to record each waypoint; "
            "type q then Enter to finish."
        )
        waypoints = []
        while True:
            answer = input("Record current pose? [Enter/q]: ").strip().lower()
            if answer == "q":
                break
            waypoint = make_waypoint(len(waypoints) + 1, read_all(packet_handler))
            waypoints.append(waypoint)
            xyz = waypoint["position_mm"]
            print(
                f"Waypoint {waypoint['number']}: "
                f"x={xyz['x']:.1f}, y={xyz['y']:.1f}, z={xyz['z']:.1f} mm"
            )

        if len(waypoints) < 2:
            print("Need at least two waypoints; no path file saved.")
            return

        MOTION_FOLDER.mkdir(exist_ok=True)
        path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "type": "cartesian_path",
                    "name": name,
                    "description": description,
                    "waypoints": waypoints,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"Saved {len(waypoints)} waypoints to {path}")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
