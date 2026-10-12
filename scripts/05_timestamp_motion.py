#!/usr/bin/env python3
"""Record a timestamped motion or play back a saved timestamped motion."""

from timestamp_player import play_motion
from timestamp_recorder import record_motion


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
