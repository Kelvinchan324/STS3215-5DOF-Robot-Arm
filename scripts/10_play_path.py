#!/usr/bin/env python3
"""Choose a recorded Cartesian path and interpolate it with inverse kinematics."""

import json
import math
import sys
import time
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_FOLDER))

import arm_settings as settings
from arm_helpers import (
    angles_to_positions,
    connect,
    disable_all_torque,
    enable_all_torque,
    move_positions_sync,
    positions_to_angles,
    read_all,
)
from robot_math import forward_kinematics, inverse_kinematics, tool_pitch_degrees
from terminal_keys import RawTerminal

MOTION_FOLDER = PROJECT_FOLDER / "motions"


def load_path(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("type") != "cartesian_path" or data.get("name") != path.stem:
        raise ValueError(f"{path.name} is not a valid Cartesian path file")

    waypoints = data.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError(f"{path.name} must contain at least two waypoints")
    for waypoint in waypoints:
        point = waypoint.get("position_mm", {})
        for axis in ("x", "y", "z"):
            if axis not in point or not math.isfinite(float(point[axis])):
                raise ValueError(f"{path.name}: waypoint is missing a valid {axis} value")
            point[axis] = float(point[axis])
        pitch = float(waypoint["tool_pitch_degrees"])
        if not math.isfinite(pitch):
            raise ValueError(f"{path.name}: invalid tool pitch")
        angles = waypoint.get("angles_degrees", {})
        if "gripper" not in angles:
            raise ValueError(f"{path.name}: waypoint is missing gripper angle")
        gripper_angle = float(angles["gripper"])
        if not math.isfinite(gripper_angle):
            raise ValueError(f"{path.name}: invalid gripper angle")
        waypoint["tool_pitch_degrees"] = pitch
        waypoint["angles_degrees"]["gripper"] = gripper_angle
    return data, waypoints


def find_paths():
    paths = []
    for path in sorted(MOTION_FOLDER.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if data.get("type") == "cartesian_path":
            paths.append((path, data))
    return paths


def choose_path(paths):
    print("Recorded Cartesian paths:")
    for index, (path, data) in enumerate(paths, start=1):
        count = len(data.get("waypoints", []))
        print(
            f"  {index}. {path.stem} — "
            f"{data.get('description') or '(no description)'} ({count} waypoints)"
        )

    selection = input("Enter a number or path name (q to quit): ").strip()
    if selection.lower() == "q":
        return None
    if selection.isdigit() and 1 <= int(selection) <= len(paths):
        return paths[int(selection) - 1][0]
    for path, _data in paths:
        if path.stem == selection:
            return path
    raise ValueError(f"No path selected as {selection!r}")


def interpolate_segment(start, end, reference_angles):
    start_point = start["position_mm"]
    end_point = end["position_mm"]
    distance = math.dist(
        tuple(float(start_point[axis]) for axis in ("x", "y", "z")),
        tuple(float(end_point[axis]) for axis in ("x", "y", "z")),
    )
    pitch_delta = end["tool_pitch_degrees"] - start["tool_pitch_degrees"]
    gripper_delta = (
        end["angles_degrees"]["gripper"] - start["angles_degrees"]["gripper"]
    )
    steps = max(
        1,
        math.ceil(distance / settings.PATH_PLAY_MAX_STEP_MM),
        math.ceil(
            distance
            / (
                settings.PATH_PLAY_SPEED_MM_PER_SECOND
                * settings.PATH_PLAY_INTERVAL_SECONDS
            )
        ),
        math.ceil(abs(pitch_delta) / 2.0),
        math.ceil(abs(gripper_delta) / 2.0),
    )

    for index in range(1, steps + 1):
        fraction = index / steps
        x = start_point["x"] + (end_point["x"] - start_point["x"]) * fraction
        y = start_point["y"] + (end_point["y"] - start_point["y"]) * fraction
        z = start_point["z"] + (end_point["z"] - start_point["z"]) * fraction
        pitch = start["tool_pitch_degrees"] + pitch_delta * fraction
        target_angles = inverse_kinematics(
            x, y, z, pitch, reference_angles
        )
        target_angles["gripper"] = (
            start["angles_degrees"]["gripper"] + gripper_delta * fraction
        )
        minimum, maximum = settings.ANGLE_LIMITS["gripper"]
        if not minimum <= target_angles["gripper"] <= maximum:
            raise ValueError("Path movement exceeds the gripper joint limit")
        yield target_angles
        reference_angles = target_angles


def main():
    if not settings.CAD_ZERO_ALIGNMENT_CONFIRMED:
        raise RuntimeError(
            "Cartesian path playback is disabled until arm_settings.py confirms "
            "that the Pi angle zeroes match the CAD home pose"
        )
    paths = find_paths()
    if not paths:
        print("No Cartesian paths found. Create one with scripts/09_record_path.py.")
        return
    path = choose_path(paths)
    if path is None:
        print("Path playback cancelled.")
        return
    data, waypoints = load_path(path)
    print(
        f"\nSelected: {data['name']} — "
        f"{data.get('description') or '(no description)'}"
    )
    print(
        "Confirm the website hardware middles match ZERO_TICKS and DIRECTIONS "
        "in arm_settings.py."
    )
    input("Clear and support the arm, then press Enter to play (Space = E-stop)...")

    port, packet_handler = connect()
    try:
        current_angles = positions_to_angles(read_all(packet_handler))
        enable_all_torque(packet_handler)
        x, y, z = forward_kinematics(current_angles)
        current_point = {
            "position_mm": {"x": x, "y": y, "z": z},
            "tool_pitch_degrees": tool_pitch_degrees(current_angles),
            "angles_degrees": current_angles,
        }
        segments = [(current_point, waypoints[0])]
        segments.extend(zip(waypoints, waypoints[1:]))
        stopped = False

        print("Press Space during playback to disable torque on all servos.")
        with RawTerminal() as terminal:
            for segment_number, (start, end) in enumerate(segments, start=1):
                for target_angles in interpolate_segment(start, end, current_angles):
                    key = terminal.read_key(settings.PATH_PLAY_INTERVAL_SECONDS)
                    if key == " ":
                        disable_all_torque(packet_handler)
                        stopped = True
                        print("\nEMERGENCY STOP: torque released on all servos.")
                        break
                    if key == "q":
                        stopped = True
                        print("\nPath playback cancelled.")
                        break

                    move_positions_sync(
                        packet_handler,
                        angles_to_positions(target_angles),
                        settings.JOINT_NAMES,
                        speed=settings.KEYBOARD_MOVE_SPEED,
                        acceleration=settings.KEYBOARD_MOVE_ACCELERATION,
                    )
                    current_angles = target_angles
                if stopped:
                    break
                print(f"Reached path segment {segment_number}/{len(segments)}")
    finally:
        port.closePort()
    if not stopped:
        print("Path playback finished.")


if __name__ == "__main__":
    main()
