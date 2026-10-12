#!/usr/bin/env python3
"""Release torque, capture a start pose, then record motion until q is pressed."""

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
from arm_helpers import connect, disable_all_torque, positions_to_angles, read_all

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


if __name__ == "__main__":
    record_motion()
