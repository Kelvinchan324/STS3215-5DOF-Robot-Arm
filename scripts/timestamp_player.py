#!/usr/bin/env python3
"""Choose/play a recorded motion; press Space during playback to release torque."""

import json
import sys
import time
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_FOLDER))

import arm_settings as settings
from arm_helpers import (
    connect,
    disable_all_torque,
    enable_all_torque,
    move_all_sync,
)
from terminal_keys import RawTerminal

MOTION_FOLDER = PROJECT_FOLDER / "motions"


def find_motion_files():
    files = []
    for path in sorted(MOTION_FOLDER.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            files.append(path)
            continue
        if not (
            isinstance(data, dict)
            and data.get("type") in ("cartesian_path", "pose_sequence")
        ):
            files.append(path)
    return files


def load_motion(path):
    motion = json.loads(path.read_text(encoding="utf-8"))
    if motion.get("name") != path.stem:
        raise ValueError(f"{path.name}: motion name does not match its file name")

    steps = motion.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError(f"{path.name}: no recorded steps to play")

    poses = []
    initial = motion.get("initial_position")
    if initial:
        initial_positions = initial.get("positions_raw")
        if not isinstance(initial_positions, dict):
            raise ValueError(f"{path.name}: invalid initial_position")
        poses.append(
            {
                "positions": validate_positions(initial_positions, path.name),
                "elapsed_seconds": None,
            }
        )

    for step in steps:
        positions = step.get("positions_raw", step.get("positions"))
        if not isinstance(positions, dict):
            raise ValueError(f"{path.name}: a step is missing raw positions")
        elapsed = step.get("elapsed_seconds")
        if elapsed is not None:
            elapsed = float(elapsed)
            if elapsed < 0:
                raise ValueError(f"{path.name}: elapsed time cannot be negative")
        poses.append(
            {
                "positions": validate_positions(positions, path.name),
                "elapsed_seconds": elapsed,
            }
        )

    interval = float(motion.get("sampling_interval_seconds", 0.1))
    if interval < 0:
        raise ValueError(f"{path.name}: sampling interval cannot be negative")
    return motion, poses, interval


def validate_positions(positions, filename):
    if set(positions) != set(settings.JOINT_NAMES):
        raise ValueError(f"{filename}: each pose must contain all five named joints")
    checked = {}
    for joint in settings.JOINT_NAMES:
        try:
            position = int(positions[joint])
        except (TypeError, ValueError) as error:
            raise ValueError(f"{filename}: invalid position for {joint}") from error
        if not 0 <= position <= 4095:
            raise ValueError(f"{filename}: {joint} position must be between 0 and 4095")
        checked[joint] = position
    return checked


def choose_motion(files):
    print("Recorded motions:")
    for number, path in enumerate(files, start=1):
        try:
            motion = json.loads(path.read_text(encoding="utf-8"))
            description = motion.get("description", "")
            step_count = len(motion.get("steps", []))
        except (json.JSONDecodeError, OSError):
            description = "Could not read this file"
            step_count = 0
        print(f"  {number}. {path.stem} — {description} ({step_count} samples)")

    selection = input("\nEnter a number or motion name (q to quit): ").strip()
    if selection.lower() == "q":
        return None
    if selection.isdigit():
        number = int(selection)
        if 1 <= number <= len(files):
            return files[number - 1]
    for path in files:
        if path.stem == selection:
            return path
    raise ValueError(f"No motion selected as {selection!r}")


def play_motion():
    files = find_motion_files()
    if not files:
        print("No timestamped motion files found. Record one with script 05.")
        return

    path = choose_motion(files)
    if path is None:
        print("Playback cancelled.")
        return

    motion, poses, interval = load_motion(path)
    print(f"\nSelected: {motion['name']}")
    print(f"Description: {motion.get('description') or '(none)'}")
    print(f"Recorded samples: {len(motion['steps'])}")
    print("Playback uses each saved sample's raw servo positions.")
    input("Support/clear the arm, then press Enter to play (Ctrl+C to cancel)...")

    port, packet_handler = connect()
    try:
        enable_all_torque(packet_handler)
        previous_elapsed = None
        with RawTerminal() as terminal:
            for index, pose in enumerate(poses, start=1):
                if terminal.read_key(0) == " ":
                    print("\nEMERGENCY STOP: releasing torque on all servos.")
                    disable_all_torque(packet_handler)
                    break

                elapsed = pose["elapsed_seconds"]
                if index > 1:
                    if elapsed is not None and previous_elapsed is not None:
                        delay = max(0.0, elapsed - previous_elapsed)
                    else:
                        delay = interval
                    deadline = time.monotonic() + delay
                    while time.monotonic() < deadline:
                        key = terminal.read_key(
                            min(0.02, deadline - time.monotonic())
                        )
                        if key == " ":
                            print("\nEMERGENCY STOP: releasing torque on all servos.")
                            disable_all_torque(packet_handler)
                            return

                move_all_sync(
                    packet_handler,
                    pose["positions"],
                    speed=settings.KEYBOARD_MOVE_SPEED,
                    acceleration=settings.KEYBOARD_MOVE_ACCELERATION,
                )
                if elapsed is not None:
                    previous_elapsed = elapsed
                print(
                    f"\rPlaying pose {index}/{len(poses)} (Space = E-stop)",
                    end="",
                    flush=True,
                )
    except KeyboardInterrupt:
        print("\nPlayback stopped.")
    finally:
        port.closePort()
    print("\nPlayback finished.")


if __name__ == "__main__":
    play_motion()
