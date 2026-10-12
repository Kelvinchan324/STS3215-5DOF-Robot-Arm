#!/usr/bin/env python3
"""Record paper waypoints and play them back in order."""

from datetime import datetime
import json
import math
import os
import re
import sys
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_FOLDER))

import arm_settings as settings
from arm_helpers import (
    angle_to_position,
    connect,
    disable_all_torque,
    enable_all_torque,
    move_all_sync,
    wait_for_servos_or_estop,
)
from terminal_keys import RawTerminal

MOTION_FOLDER = PROJECT_FOLDER / "motions"
NAME_PATTERN = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,47}$")


def parse_point(raw_values, input_type):
    """Parse one five-joint point and return validated raw positions."""
    return parse_point_values(raw_values.split(), input_type)


def parse_point_values(values, input_type):
    """Convert stored input values into raw servo positions."""
    if len(values) != len(settings.JOINT_NAMES):
        raise ValueError(f"Enter exactly {len(settings.JOINT_NAMES)} values")

    positions = {}
    for name, value in zip(settings.JOINT_NAMES, values):
        if input_type == "raw":
            try:
                position = int(value)
            except ValueError as error:
                raise ValueError(f"{name} raw position must be a whole number") from error
            if not 0 <= position <= 4095:
                raise ValueError(f"{name} raw position must be between 0 and 4095")
        elif input_type == "angles":
            try:
                angle = float(value)
            except ValueError as error:
                raise ValueError(f"{name} angle must be a number") from error
            if not math.isfinite(angle):
                raise ValueError(f"{name} angle must be finite")
            position = angle_to_position(name, angle)
        else:
            raise ValueError(f"Unknown point input type: {input_type}")
        positions[name] = position
    return positions


def save_sequence(path, sequence):
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    lines = ["{"]
    fields = list(sequence.items())
    for field_index, (key, value) in enumerate(fields):
        is_last_field = field_index == len(fields) - 1
        if key == "points":
            lines.append('  "points": [')
            for point_index, point in enumerate(value):
                comma = "," if point_index < len(value) - 1 else ""
                lines.append(
                    "    "
                    + json.dumps(point, separators=(", ", ": "))
                    + comma
                )
            lines.append("  ]" + ("" if is_last_field else ","))
        else:
            comma = "" if is_last_field else ","
            lines.append(
                f"  {json.dumps(key)}: "
                f"{json.dumps(value, separators=(', ', ': '))}{comma}"
            )
    lines.append("}")
    temporary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    os.replace(temporary_path, path)


def create_sequence():
    name = input("Waypoint sequence name (letters, numbers, _ or -): ").strip()
    if not NAME_PATTERN.fullmatch(name):
        raise ValueError("Use 1-48 letters, numbers, underscores, or hyphens")
    path = MOTION_FOLDER / f"{name}.json"
    if path.exists():
        raise FileExistsError(f"{path} already exists; choose a new name")

    description = input("Waypoint sequence description: ").strip()
    print("\nChoose the values copied from your notes:")
    print("  1. Raw servo positions (0-4095)")
    print("  2. Joint angles in degrees")
    choice = input("Input type: ").strip()
    if choice not in ("1", "2"):
        raise ValueError("Choose 1 for raw positions or 2 for joint angles")
    input_type = "raw" if choice == "1" else "angles"
    order = ", ".join(
        f"ID {settings.SERVO_IDS[joint]} {joint}"
        for joint in settings.JOINT_NAMES
    )
    print(f"\nEnter five values in this order: {order}")
    if input_type == "angles":
        for joint_name in settings.JOINT_NAMES:
            minimum, maximum = settings.ANGLE_LIMITS[joint_name]
            print(
                f"  ID {settings.SERVO_IDS[joint_name]} {joint_name}: "
                f"{minimum:g} to {maximum:g} degrees"
            )
    print(
        "Enter one pose per line. Type 'play' after your last point "
        "to save and start playback. Type 'q' to save and return to the menu."
    )

    points = []
    play_after_save = False
    while True:
        raw_values = input(f"Point {len(points) + 1}: ").strip()
        command = raw_values.lower()
        if command in ("play", "p"):
            play_after_save = True
            break
        if command == "q":
            break
        try:
            positions = parse_point(raw_values, input_type)
        except ValueError as error:
            print(f"Invalid point: {error}. Please try again.")
            continue
        points.append(
            {
                "input_values": raw_values.split(),
            }
        )
        raw_point = " ".join(
            str(positions[name]) for name in settings.JOINT_NAMES
        )
        print(f"  Converted to raw ticks: {raw_point}")

    if len(points) < 2:
        print("Enter at least two waypoints before saving or playing a sequence.")
        return None

    sequence = {
        "schema_version": 1,
        "type": "pose_sequence",
        "name": name,
        "description": description,
        "input_type": input_type,
        "joint_order": [
            {"servo_id": settings.SERVO_IDS[joint], "joint": joint}
            for joint in settings.JOINT_NAMES
        ],
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "points": points,
    }
    MOTION_FOLDER.mkdir(exist_ok=True)
    save_sequence(path, sequence)
    print(f"Saved {len(points)} points to {path}")
    return path if play_after_save else None


def load_sequence(path):
    sequence = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(sequence, dict) or sequence.get("type") != "pose_sequence":
        raise ValueError(f"{path.name} is not a valid waypoint sequence")
    points = sequence.get("points")
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError(f"{path.name} must contain at least two points")

    input_type = sequence.get("input_type")
    if input_type not in ("angles", "raw"):
        raise ValueError(f"{path.name}: input_type must be 'angles' or 'raw'")

    checked_points = []
    for index, point in enumerate(points, start=1):
        if not isinstance(point, dict):
            raise ValueError(f"{path.name}: point {index} must be an object")
        input_values = point.get("input_values")
        if input_values is not None:
            try:
                checked = parse_point_values(input_values, input_type)
            except (TypeError, ValueError) as error:
                raise ValueError(f"{path.name}: point {index}: {error}") from error
        else:
            checked = _load_legacy_positions(path, index, point)
        checked_points.append(checked)
    return sequence, checked_points


def _load_legacy_positions(path, index, point):
    """Allow older sequence files to keep playing while users migrate them."""
    positions = point.get("positions_raw")
    if not isinstance(positions, dict) or set(positions) != set(settings.JOINT_NAMES):
        raise ValueError(
            f"{path.name}: point {index} needs input_values or all five raw positions"
        )
    checked = {}
    for joint in settings.JOINT_NAMES:
        try:
            position = int(positions[joint])
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{path.name}: point {index} has an invalid {joint} position"
            ) from error
        if not 0 <= position <= 4095:
            raise ValueError(
                f"{path.name}: point {index} {joint} must be between 0 and 4095"
            )
        checked[joint] = position
    return checked


def find_sequences():
    sequences = []
    for path in sorted(MOTION_FOLDER.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if isinstance(data, dict) and data.get("type") == "pose_sequence":
            sequences.append((path, data))
    return sequences


def play_sequence(path):
    sequence, points = load_sequence(path)
    print(
        f"\nSelected: {path.stem} — "
        f"{sequence.get('description') or '(no description)'}"
    )
    print(f"Points: {len(points)}; input values were {sequence.get('input_type', 'unknown')}.")
    input("Support and clear the arm, then press Enter to play (Space = E-stop)...")

    port, packet_handler = connect()
    try:
        enable_all_torque(packet_handler)
        servo_ids = [settings.SERVO_IDS[name] for name in settings.JOINT_NAMES]
        with RawTerminal() as terminal:
            for index, positions in enumerate(points, start=1):
                if terminal.read_key(0) == " ":
                    print("\nEMERGENCY STOP: releasing torque on all servos.")
                    disable_all_torque(packet_handler)
                    return
                move_all_sync(
                    packet_handler,
                    positions,
                    speed=settings.POSE_SEQUENCE_MOVE_SPEED,
                    acceleration=settings.POSE_SEQUENCE_MOVE_ACCELERATION,
                )
                print(
                    f"\rMoving to point {index}/{len(points)} (Space = E-stop)",
                    end="",
                    flush=True,
                )
                if not wait_for_servos_or_estop(packet_handler, servo_ids, terminal):
                    return
        print("\nWaypoint sequence finished.")
    except KeyboardInterrupt:
        print("\nSequence playback interrupted.")
    finally:
        port.closePort()


def main():
    while True:
        print("\nWaypoint record and play")
        print("  1. Record waypoints from written-down values")
        print("  2. Play a saved waypoint sequence")
        print("  q. Quit")
        choice = input("Choose an action: ").strip().lower()
        if choice == "q":
            return
        if choice == "1":
            sequence_path = create_sequence()
            if sequence_path is not None:
                play_sequence(sequence_path)
        elif choice == "2":
            sequences = find_sequences()
            if not sequences:
                print("No waypoint sequences found. Create one first.")
                continue
            for index, (path, data) in enumerate(sequences, start=1):
                print(
                    f"  {index}. {path.stem} — "
                    f"{data.get('description') or '(no description)'} "
                    f"({len(data.get('points', []))} points)"
                )
            selection = input("Enter a number or q to cancel: ").strip().lower()
            if selection == "q":
                continue
            if not selection.isdigit() or not 1 <= int(selection) <= len(sequences):
                print("Please choose one of the listed numbers.")
                continue
            play_sequence(sequences[int(selection) - 1][0])
        else:
            print("Please enter 1, 2, or q.")


if __name__ == "__main__":
    try:
        main()
    except (EOFError, KeyboardInterrupt):
        print("\nWaypoint tool stopped.")
