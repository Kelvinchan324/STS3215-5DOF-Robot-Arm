# STS3215 Robot Arm — Raspberry Pi Student Guide

Use these five Python programs to read, move and record a real robot arm with
**STS3215 servos and a Waveshare ESP32 controller** connected to a Raspberry Pi by USB.

## Which repository should I use?

| Repository | What you can learn and do |
| --- | --- |
| [STS3215-5DOF-Robot-Arm](https://github.com/Kelvinchan324/STS3215-5DOF-Robot-Arm) — this repository | Read motor positions, command joint movements, record motions and play waypoint sequences in Python. |
| [robot-arm-simulation](https://github.com/Kelvinchan324/robot-arm-simulation) | Explore the arm in 3D, forward and inverse kinematics, transformations and keyboard control in a browser. |

**[Open the online simulation](https://kelvinchan324.github.io/robot-arm-simulation/)**
to explore without installing anything or connecting a physical arm.

## 1. Clone both repositories onto your Raspberry Pi

Open a terminal on Raspberry Pi OS. Install Git and the Python tools:

```sh
sudo apt update
sudo apt install git python3 python3-venv
```

Create a folder for your work and download both repositories:

```sh
mkdir -p ~/robot-arm-lab
cd ~/robot-arm-lab
git clone https://github.com/Kelvinchan324/STS3215-5DOF-Robot-Arm.git
git clone https://github.com/Kelvinchan324/robot-arm-simulation.git
```

You now have two separate project folders inside `~/robot-arm-lab`.
Clone each repository only once; use `git pull` inside its folder for later updates.

## 2. Set up the Python exercises

Create a Python environment and install the serial communication package:

```sh
cd ~/robot-arm-lab/STS3215-5DOF-Robot-Arm
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

The servo SDK is already included in `scservo_sdk/`.
When you open a new terminal, return to this folder and run
`source .venv/bin/activate` again before running the exercises.

### Connect and configure the arm

Connect the Waveshare ESP32 controller to the Pi by USB, prepare its serial
forwarding mode and connect the servo power supply. Close any other program
using the same serial port, including a connected browser controller.

Edit [arm_settings.py](arm_settings.py) to match your arm:

| Setting | Meaning |
| --- | --- |
| `SERIAL_PORT` | USB serial device; default `/dev/ttyUSB0`. |
| `BAUD_RATE` | Serial connection speed; default `115200`. |
| `SERVO_IDS` | Base **1**, shoulder **2**, elbow **3**, wrist **4**, gripper **5**. |
| `ZERO_TICKS` | Raw position corresponding to 0° for each joint. Check the defaults of 2048 against your arm. |
| `DIRECTIONS` and `ANGLE_LIMITS` | Each joint's positive rotation direction and allowed angle range. |
| Speed and acceleration settings | Separate settings for single moves, timestamp playback and waypoint playback. |

**Support the arm before releasing torque.** Reading positions and recording
timestamp motions release torque, so the arm can fall under gravity. During
movement/playback, **Space** also releases torque. This software stop depends
on the serial connection; keep the physical stop accessible.

## 3. Try the five scripts

Run these commands from `STS3215-5DOF-Robot-Arm` with the Python environment
activated. Whenever you enter five values, use this order:
**base → shoulder → elbow → wrist → gripper**.

### Read positions — `01_read_positions.py`

Connects to the controller, releases motor torque and reads all five servos
once. Prints each servo's raw position in ticks and its angle calculated from
your calibration settings. Use it to check the connection and note a pose.

```sh
python scripts/01_read_positions.py
```

### Move servos — `02_move_servo.py`

Asks for five raw target positions from **0 to 4095**. Checks the input and,
after confirmation, sends the targets together in one synchronized command.
Raw ticks are motor positions, not degrees.

```sh
python scripts/02_move_servo.py
```

### Move joint angles — `03_move_joint_angle.py`

Asks for five target angles in **degrees**. Checks the configured joint limits,
converts the angles to raw motor positions using `arm_settings.py`, then sends
a synchronized move after confirmation.

```sh
python scripts/03_move_joint_angle.py
```

### Timestamp motion — `05_timestamp_motion.py`

Choose **1** to record a motion by moving the supported arm by hand with torque
released. The program records positions, angles and timestamps every **0.1
seconds** by default. Type `q` and press Enter to stop and save in `motions/`.
Choose **2** to replay a recording using its saved raw positions and timing.
Use a new name for each recording.

```sh
python scripts/05_timestamp_motion.py
```

### Waypoint sequence — `06_waypoint_sequence.py`

Choose **1** to enter a sequence of poses using either raw ticks or joint
angles. Enter at least two poses, one per line. Type `play` to save and play
immediately, or `q` to save without playing. Choose **2** to play a saved
sequence. Files are stored in `motions/`; angle inputs use the current calibration
when played.

```sh
python scripts/06_waypoint_sequence.py
```

Timestamp motion captures movement over time; waypoint sequences follow a
list of poses you enter. The script numbers are kept from the original course,
so there is no script 04.

## 4. Explore the 3D simulation

The [simulation repository](https://github.com/Kelvinchan324/robot-arm-simulation)
contains a browser-based MuJoCo arm model built from the Fusion 360 assembly.
The 3D view, joint angles, tool position and transformation matrix help you
connect the equations to the arm's movement:

- **Forward kinematics:** change joint angles and observe the tool position.
- **Inverse kinematics:** enter an XYZ target or pick a point in 3D; the solver
  tries to reach it within the joint limits.
- **Keyboard control:** W/S moves up/down, Left/Right changes yaw, Up/Down
  moves forward/back, A/D adjusts pitch, and holding J/L slowly opens/closes
  the gripper. Release J/L to stop changing its angle; full travel takes about
  six seconds at the default rate, with lower hardware limits respected.
  Enable keyboard control first; translation holds the tool pitch.

### Run your cloned simulation locally

In another terminal on your Pi, run:

```sh
cd ~/robot-arm-lab/robot-arm-simulation
python3 -m http.server 8000 --bind 127.0.0.1 --directory dist
```

Open **http://localhost:8000** in a browser on the same Pi. Keep the terminal
running while using the simulation; press **Ctrl+C** to stop the server.
The ready-to-run files are in `dist/`, so no Node.js installation or build step
is needed to view them. You can also use the
[hosted simulation](https://kelvinchan324.github.io/robot-arm-simulation/).

### Use the simulation alongside the real arm

The simulation project also includes USB serial keyboard control and a 3D view
that follows measured motor positions. Follow its
[Web Serial setup guide](https://github.com/Kelvinchan324/robot-arm-simulation/blob/main/raspberry_pi/WEB_SERIAL.md)
for browser requirements, calibration and connection steps. It also has a
[Python keyboard controller guide](https://github.com/Kelvinchan324/robot-arm-simulation/blob/main/raspberry_pi/README.md).

Connecting starts with position readings; movement requires the simulation
project's measured calibration profile and explicit keyboard enabling. Settings
in this repository's `arm_settings.py` do **not** automatically configure the
simulation. Disconnect the browser before running these Python exercises,
because only one program should use the serial port at a time.

## Supporting files

| File or folder | Purpose |
| --- | --- |
| `arm_settings.py` | Serial connection, calibration, limits and motion speeds. |
| `arm_helpers.py` | Shared motor communication and raw-position/angle conversion. |
| `terminal_keys.py` | Reads the Space key during movement/playback. |
| `scservo_sdk/` | Vendor library for communicating with the servos. |
| `motions/` | Saved recordings and waypoint sequences. |
| `tests/` | Offline checks using mocked motor communication. |

To run the offline checks from this repository with its environment activated:

```sh
python -m unittest discover -s tests -v
```

These checks do not move the physical arm.
