#!/usr/bin/env python3
"""Teaching menu for the standalone robot-arm lesson scripts."""

import subprocess
import sys
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parent
SCRIPTS_FOLDER = PROJECT_FOLDER / "scripts"

LESSONS = (
    (
        "Part 1: Raspberry Pi, ESP32, and serial communication",
        "Connect to the Waveshare controller and verify servo communication.",
        (
            ("Read positions", "01_read_positions.py"),
        ),
    ),
    (
        "Part 2: Motor movement and forward kinematics",
        "Use the website-set hardware middle, try controlled moves, and calculate gripper position.",
        (
            ("Move all five motors using raw positions", "02_move_servo.py"),
            ("Move all five motors using angles", "03_move_joint_angle.py"),
            ("Visualize robot-arm kinematics", "04_forward_kinematics.py"),
        ),
    ),
    (
        "Part 3: JSON motion recording, playback, and storage",
        "Record timestamped motions or enter waypoints; JSON files are saved in motions/.",
        (
            ("Record/play a timestamped motion", "05_timestamp_motion.py"),
            ("Record/play a waypoint sequence", "06_waypoint_sequence.py"),
            ("Record Cartesian waypoints (extension)", "09_record_path.py"),
            ("Play an interpolated Cartesian path (extension)", "10_play_path.py"),
        ),
    ),
    (
        "Part 4: Block placement recording practice",
        "Record and replay placement attempts with a unique block_placement_* name.",
        (
            ("Record/play a timestamped placement motion", "05_timestamp_motion.py"),
            ("Record/play a placement waypoint sequence", "06_waypoint_sequence.py"),
        ),
    ),
    (
        "Part 5: Block attack recording practice",
        "Record and replay attack attempts with a unique block_attack_* name.",
        (
            ("Record/play a timestamped attack motion", "05_timestamp_motion.py"),
            ("Record/play an attack waypoint sequence", "06_waypoint_sequence.py"),
        ),
    ),
    (
        "Part 6: Robot arm control coding",
        "Explore keyboard-based control and the graphical user interface.",
        (
            ("Continuous keyboard control", "07_keyboard_control.py"),
            ("Open the robot-arm GUI", "08_robot_arm_gui.py"),
        ),
    ),
)


def choose(prompt, options):
    while True:
        print()
        for number, label in enumerate(options, start=1):
            print(f"  {number}. {label}")
        print("  q. Back / quit")
        answer = input(prompt).strip().lower()
        if answer == "q":
            return None
        if answer.isdigit() and 1 <= int(answer) <= len(options):
            return int(answer) - 1
        print("Please enter one of the listed numbers or q.")


def run_script(script_name):
    script_path = SCRIPTS_FOLDER / script_name
    if not script_path.is_file():
        raise FileNotFoundError(f"Lesson script not found: {script_path}")
    answer = input(
        f"Run {script_name}? Check the arm is supported first. [y/N]: "
    ).strip().lower()
    if answer != "y":
        print("Activity cancelled.")
        return
    print(f"\nStarting {script_name}. Press Ctrl+C to return if needed.\n")
    result = subprocess.run(
        [sys.executable, str(script_path)],
        cwd=PROJECT_FOLDER,
        check=False,
    )
    if result.returncode:
        print(f"\n{script_name} exited with status {result.returncode}.")
    else:
        print(f"\nFinished {script_name}.")


def main():
    print("STS3215 5-DOF Robot Arm — Teaching Menu")
    print("Choose one of the six teaching parts.")
    print("Before connecting, check arm_settings.py and support the arm.")

    try:
        while True:
            lesson_index = choose(
                "Choose a part: ", [lesson[0] for lesson in LESSONS]
            )
            if lesson_index is None:
                print("Goodbye.")
                return

            lesson_name, objective, activities = LESSONS[lesson_index]
            while True:
                print(f"\n{lesson_name}")
                print(objective)
                activity_index = choose(
                    "Choose an activity: ", [activity[0] for activity in activities]
                )
                if activity_index is None:
                    break
                run_script(activities[activity_index][1])
    except (EOFError, KeyboardInterrupt):
        print("\nExiting teaching menu.")


if __name__ == "__main__":
    main()
