# STS3215 robot arm: Raspberry Pi teaching scripts

Five standalone programs for the Waveshare ESP32 controller and STS3215 servos.
Students clone this repository onto their own Raspberry Pi. The servo SDK is
included; the only additional Python package is `pyserial`.

## Install on the Raspberry Pi

```sh
git clone https://github.com/Kelvinchan324/STS3215-5DOF-Robot-Arm.git
cd STS3215-5DOF-Robot-Arm
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

If `venv` is unavailable on Raspberry Pi OS, install it with
`sudo apt install python3-venv`, then repeat the environment setup.
Activate `.venv` again when opening a new terminal.

## Configure the arm

Edit `arm_settings.py` before movement:

- `SERIAL_PORT`: defaults to `/dev/ttyUSB0`.
- `BAUD_RATE`: defaults to `115200`; match the controller's serial connection.
- Servo IDs: base yaw 1, shoulder 2, elbow 3, wrist 4, gripper 5.
- `ZERO_TICKS`, `DIRECTIONS` and `ANGLE_LIMITS`: check against each physical arm.
  The existing zero defaults are 2048; they are not measurements of a student's arm.
- `MOVE_SPEED` / `MOVE_ACCELERATION`: raw and joint-angle moves.
- `TIMESTAMP_MOVE_SPEED` / `TIMESTAMP_MOVE_ACCELERATION`: timestamp playback.
- `POSE_SEQUENCE_MOVE_SPEED` / `POSE_SEQUENCE_MOVE_ACCELERATION`: waypoints.
- `RECORD_INTERVAL_SECONDS`: timestamp recording interval, default 0.1 seconds.

Prepare the board's serial forwarding and servo power before connecting.
Close browser controllers and other serial programs before running a Pi script.
Use a terminal window for playback and its Space-key stop control.

## The five programs

Run these commands from the repository folder:

| Program | Command | Purpose |
| --- | --- | --- |
| Read positions | `python scripts/01_read_positions.py` | Release torque and display all five raw positions and joint angles. |
| Move servos | `python scripts/02_move_servo.py` | Enter five raw targets and send a synchronized move. |
| Move joint angles | `python scripts/03_move_joint_angle.py` | Enter five angles, validate configured limits and move all joints. |
| Timestamp motion | `python scripts/05_timestamp_motion.py` | Record manually posed motion or play a saved recording. |
| Waypoint sequence | `python scripts/06_waypoint_sequence.py` | Enter raw/angle waypoints and play them in order. |

The existing filenames are retained, including the gap at 04. `scripts/`
contains only these five files. Timestamp recording and playback helpers are
included inside script 05.

### Timestamp motion

Choose **1** to record or **2** to play. Recording releases torque, reads the
starting pose, and asks for a name and description. Move the supported arm by
hand; type `q` and press Enter to stop. Recordings are checkpointed every two
seconds and saved in `motions/` with raw positions, angles and timestamps.
Playback follows the saved raw positions and elapsed times. Use a new filename
for each recording; existing recordings with samples are not overwritten.

### Waypoint sequence

Choose raw positions or joint angles, then enter one five-value pose per line
in the displayed servo-ID order. At least two poses are needed. Type `play` to
save and play immediately, or `q` to save without playing. The player converts
stored angle inputs using the current settings. Existing saved motion and
waypoint files remain in `motions/`.

**Torque release:** script 01 and timestamp recording release all servo torque.
Support the arm before starting them. Space during movement/playback also
releases torque, so the arm can fall under gravity. This software stop depends
on the serial link; use the arm's physical stop when needed.

## Repository contents

```text
scripts/           The five student programs above
scservo_sdk/       Required vendor servo communication library
motions/           Saved timestamp motions and waypoint sequences
arm_settings.py    Connection, calibration, limits and speed settings
arm_helpers.py     Shared serial, raw/angle conversion and motion helpers
terminal_keys.py   Terminal input for Space-key stopping during movement
requirements.txt  Python dependency
tests/             Offline checks for the retained programs
```

The separate [robot arm simulation repository](https://github.com/Kelvinchan324/robot-arm-simulation)
hosts the [online simulation and keyboard controller](https://kelvinchan324.github.io/robot-arm-simulation/).
This Pi repository contains no kinematics solver, GUI or web preview.
Its joint-angle conversion is a servo calibration calculation, not an XYZ solver.

## Offline checks

```sh
python -m unittest discover -s tests -v
```

These tests use mocked motor communication; they do not move the physical arm.
