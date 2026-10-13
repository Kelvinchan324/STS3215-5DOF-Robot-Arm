#!/usr/bin/env python3
"""Record or play timestamped raw-servo motions."""

from datetime import datetime
import json
import os
import re
import select
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
    positions_to_angles,
    read_all,
)
from terminal_keys import RawTerminal

MOTION_FOLDER = PROJECT_FOLDER / "motions"
NAME_PATTERN = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,47}$")
SAVE_INTERVAL_SECONDS = 2.0



def timestamp_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def save_motion(path, motion):
    """Replace the JSON atomically so a stopped recording stays readable."""
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(motion, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary_path, path)


def confirm(prompt):
    return input(prompt).strip().lower() != "q"


def stop_requested():
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(readable and sys.stdin.readline().strip().lower() == "q")


def record_motion():
    print(
        f"Connecting to {settings.SERIAL_PORT} at {settings.BAUD_RATE} baud. "
        "The arm may move when torque is released."
    )
    input("Support the arm, clear the area, then press Enter to continue...")

    port, packet_handler = connect()
    try:
        disable_all_torque(packet_handler)
        print("Torque released on all configured servos.")

        initial_raw = read_all(packet_handler)
        initial_pose = {
            "timestamp": timestamp_now(),
            "positions_raw": initial_raw,
            "angles_degrees": positions_to_angles(initial_raw),
        }
        print("\nInitial position (torque is released):")
        for name in settings.JOINT_NAMES:
            print(
                f"  {name:12s} raw={initial_pose['positions_raw'][name]:4d} "
                f"angle={initial_pose['angles_degrees'][name]:7.2f} deg"
            )

        if not confirm("\nPress Enter to confirm this start position (q to quit): "):
            print("Recording cancelled.")
            return

        name = input("Motion name (letters, numbers, _ or -): ").strip()
        if not NAME_PATTERN.fullmatch(name):
            raise ValueError("Use 1-48 letters, numbers, underscores, or hyphens")
        description = input("Motion description: ").strip()
        path = MOTION_FOLDER / f"{name}.json"

        if path.exists():
            existing = json.loads(path.read_text(encoding="utf-8"))
            if existing.get("name") != name or not isinstance(
                existing.get("steps"), list
            ):
                raise ValueError(f"{path} is not a valid motion file for {name!r}")
            if existing["steps"]:
                raise FileExistsError(
                    f"{path} already contains recorded steps; choose a new name"
                )
        else:
            existing = {"name": name, "description": description, "steps": []}

        print(f"\nFile: {path}")
        print(f"Description: {description or '(none)'}")
        if not confirm("Press Enter to confirm and start recording (q to cancel): "):
            print("Recording cancelled.")
            return

        motion = {
            "name": name,
            "description": description,
            "sampling_interval_seconds": settings.RECORD_INTERVAL_SECONDS,
            "initial_position": initial_pose,
            "steps": existing["steps"],
        }
        MOTION_FOLDER.mkdir(exist_ok=True)
        save_motion(path, motion)

        print(
            f"Recording every {settings.RECORD_INTERVAL_SECONDS:g} seconds. "
            "Move the arm by hand; press q then Enter to stop."
        )
        start_time = time.monotonic()
        last_save_time = start_time
        sample_number = 0
        try:
            while True:
                loop_start = time.monotonic()
                raw_positions = read_all(packet_handler)
                elapsed = time.monotonic() - start_time
                angles = positions_to_angles(raw_positions)
                motion["steps"].append(
                    {
                        "timestamp": timestamp_now(),
                        "elapsed_seconds": round(elapsed, 3),
                        "positions": raw_positions,
                        "positions_raw": raw_positions,
                        "angles_degrees": angles,
                        "duration_seconds": settings.RECORD_INTERVAL_SECONDS,
                    }
                )
                sample_number += 1
                if time.monotonic() - last_save_time >= SAVE_INTERVAL_SECONDS:
                    save_motion(path, motion)
                    last_save_time = time.monotonic()
                print(
                    f"\rRecorded {sample_number} samples "
                    f"({elapsed:.1f} s). Type q and press Enter to stop.",
                    end="",
                    flush=True,
                )
                if stop_requested():
                    break
                time.sleep(
                    max(
                        0.0,
                        settings.RECORD_INTERVAL_SECONDS
                        - (time.monotonic() - loop_start),
                    )
                )
        except KeyboardInterrupt:
            print("\nRecording stopped.")
        finally:
            save_motion(path, motion)

        print(f"Saved {sample_number} samples to {path}")
    finally:
        port.closePort()


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
                    speed=settings.TIMESTAMP_MOVE_SPEED,
                    acceleration=settings.TIMESTAMP_MOVE_ACCELERATION,
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


def main():
    while True:
        print("\nTimestamped motion record and play")
        print("  1. Record a motion with timestamps")
        print("  2. Play a saved motion")
        print("  q. Quit")
        choice = input("Choose an action: ").strip().lower()
        if choice == "q":
            return
        if choice == "1":
            record_motion()
        elif choice == "2":
            play_motion()
        else:
            print("Please enter 1, 2, or q.")


if __name__ == "__main__":
    try:
        main()
    except (EOFError, KeyboardInterrupt):
        print("\nTimestamped motion tool stopped.")
