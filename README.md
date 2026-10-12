# STS3215 robot arm: teaching project

Open the local interactive 3D arm website:

```sh
python3 scripts/12_3d_arm_viewer.py
# Or connect the current motors for read-only live poses:
python3 scripts/12_3d_arm_viewer.py --live
```

Visit `http://localhost:8080` in Chromium on the Raspberry Pi. Drag to orbit,
scroll to zoom, and use the five joint angle sliders to explore poses. Click
**Read motors** or enable **Follow motors** when running with `--live`.
The website displays XYZ in millimetres with Z upward and the base reference
point as origin. It uses the CAD axes and saved home calibration in
`kinematics_config.json`. It sends no movement or torque commands.
Without a measured TCP it displays the wrist reference. The geometry panel
can preview an explicitly assumed axial gripper extension. A measured
`tcp.home_point_mm` in the config takes precedence over that preview.
The site assets in `web/arm-viewer/` work offline; no CDN or npm is required.

For current motor readings converted to XYZ, run
`python3 scripts/11_read_gripper_xyz.py`. See [FORWARD_KINEMATICS.md](FORWARD_KINEMATICS.md)
for the expression, reference calibration, coordinate frame, and configuration
of the gripper centre point. Without that measurement the reader labels its
output as the wrist reference point.

Run the teaching menu from the project folder:

```sh
python3 run_robot_arm.py
```

Choose one of the six teaching parts, then choose an activity. Each activity
starts its own script and asks for confirmation before it runs. The examples
remain separate and can also be run directly. The code uses the local
`scservo_sdk/` library and follows the examples in `servo_example/`.

```text
scservo_sdk/                 Feetech SDK (leave as provided)
servo_example/                Your original SDK examples (leave as provided)
arm_settings.py               Serial port, servo IDs, angle offsets, arm lengths
arm_helpers.py                Small shared SDK and angle-conversion helpers
robot_math.py                 Forward and inverse kinematics
run_robot_arm.py              Six-part teaching menu
scripts/
  01_read_positions.py        Read all five servo positions
  02_move_servo.py            Move all five servos using raw position values
  03_move_joint_angle.py      Move all five servos using target angles
  04_forward_kinematics.py    Open interactive CAD kinematics visualization
  05_timestamp_motion.py      Record or play a timestamped motion
  06_waypoint_sequence.py     Record or play a sequence of entered waypoints
  07_keyboard_control.py      Continuous WASD and gripper control
  08_robot_arm_gui.py         Desktop controls and CAD webpage launcher
  09_record_path.py           Record manual poses as Cartesian waypoints
  10_play_path.py             Interpolate and play a Cartesian path
  timestamp_recorder.py       Recording helper used by script 05
  timestamp_player.py         Playback helper used by script 05
motions/                      Motion, path, and pose-sequence JSON files
```

## Before connecting

1. Install the SDK's Python dependency if needed:

   ```sh
   python3 -m pip install pyserial
   ```

2. Find the serial device for the Waveshare controller, for example
   `/dev/ttyUSB0`:

   ```sh
   ls /dev/ttyUSB*
   ```

3. Edit `arm_settings.py` and set the correct serial device and baud rate.
   The contest website controller's baud selector defaults to 115200. Use
   the same baud rate that successfully controls your servos on the website.
   The servo baud can be changed, so do not assume the value if it was changed
   in the website.
4. Make sure the controller/servo supply and common ground are correctly
   connected. Do not power the servos from the Raspberry Pi.
5. Keep the arm supported and clear of people when trying movement programs.
   Start at low speed and small movements.

Use the website's Calibration Mode to set each motor ID and hardware middle
position before running angle-based movement or keyboard control. On the Pi,
`ZERO_TICKS` in `arm_settings.py` is the raw position treated as 0 degrees;
it defaults to 2048 for all joints. Set each `DIRECTIONS` value to `1` or
`-1` to match that joint, and check `ANGLE_LIMITS` before moving.

Forward kinematics now uses the CAD-derived joint-axis points and unit vectors
in `CAD_JOINT_POINTS_MM` and `CAD_JOINT_AXES` in `arm_settings.py`. The
reference measurements validate as follows: J1-J2 point distance 93.40 mm
with a 42.21 mm common normal; J2-J3 120.49 mm; J3-J4 154.60 mm; and J4/J5
axes intersect even though the selected CAD reference points are 77.30 mm
apart. Coordinates from the Pi's kinematics scripts are relative to the J1
axis, in a right-handed frame with Z up. The browser webpage displays the CAD
frame (Y up) and absolute CAD coordinates.

The source measurements are in
`../robot-arm-kinematics-raspberry-pi-2026-10-09/kinematics_config.json`.
The gripper TCP is not defined in CAD. The Pi scripts therefore report the
J5 wrist-axis reference point, not the gripper tip. Add a reference point
midway between the jaws to obtain TCP coordinates.

**Important angle-zero check:** the supplied CAD calibration file lists
home positions `[2042, 3164, 1062, 1957, 2049]`, while this Pi project keeps
your existing `ZERO_TICKS` and `DIRECTIONS` unchanged, as requested. The CAD
axis model treats zero joint angles as the STEP assembly pose. Before using
keyboard Cartesian jog (`w`/`s`/`f`/`b`) or Cartesian path playback on the
physical arm, verify that the Pi's zero-angle pose matches that CAD pose; the
CAD home readings have not been applied to the Pi servo mapping. The setting
`CAD_ZERO_ALIGNMENT_CONFIRMED` is `False` by default, so these Cartesian
controls stay disabled until you verify the alignment and explicitly set it
to `True`. Start with the arm supported and the slow jog rates in
`arm_settings.py`.

Existing Cartesian path JSON files were recorded against the earlier
simplified model; re-record them before playback with this CAD-axis model.
Timestamped playback uses saved raw positions and is unaffected.

## Six teaching parts

The launcher follows the course sequence below. Activities are kept as
individual scripts so students can run or inspect one task at a time.

1. **Raspberry Pi, ESP32, and serial communication:** configure the serial
   port/baud rate, then read the five servo positions to verify the Pi can
   communicate through the Waveshare controller.
2. **Motor movement and forward kinematics:** after setting hardware middles
   in the website, try a small raw-tick movement, command a joint angle, and
   inspect the interactive CAD kinematics visualization. Confirm the arm is
   supported before movement.
3. **JSON motion recording, playback, and storage:** run script 05 and choose
   timestamped recording or playback, or run script 06 to record/play an
   ordered waypoint sequence. Both save JSON files in `motions/`.
4. **Block placement recording practice:** use script 05 for a timestamped
   placement attempt or script 06 to enter and play placement waypoints. Use a
   unique name such as `block_placement_practice_01`.
5. **Block attack recording practice:** use the same tools with a unique name
   such as `block_attack_practice_01`. Keeping the task prefix in the filename
   makes saved attempts easy to find among other JSON files.
6. **Robot arm control coding:** inspect and run the continuous keyboard
   controller and the basic GUI. The GUI's **Open CAD kinematics webpage**
   button opens the bundled reference simulator in your browser.

The Cartesian path scripts remain available as an extension activity:
script 09 records manually posed waypoints and their CAD-axis-model XYZ
positions, and script 10 interpolates and plays a selected path using
inverse kinematics. Cartesian paths are JSON files in `motions/` too, but
their `type` field distinguishes them from time-sampled motions.

To play poses written down from testing, use script 06. Choose raw servo
positions or joint angles, then enter one five-value waypoint per line in the
displayed servo-ID order. Type `play` after the last point to save the JSON
file and start playback immediately; type `q` to save and return to the menu
without playing. At least two points are required. The tool stores only the
five `input_values` for each waypoint. On playback, script 06 converts those
inputs using the current `ZERO_TICKS`, `DIRECTIONS`, and angle limits in
`arm_settings.py`; edit the `input_values` arrays to adjust poses without
manually changing calculated raw positions or angles. Space releases torque
and stops playback. Existing `pose_sequence` JSON files are still supported.
Script 04 opens the interactive forward-kinematics visualization.

Adjust sequence playback speed and acceleration independently in
`arm_settings.py` with `POSE_SEQUENCE_MOVE_SPEED` and
`POSE_SEQUENCE_MOVE_ACCELERATION`. Higher values command faster movement;
start with small increases and keep the arm supported and clear.

The individual scripts are in `scripts/` and can be run directly with
`python3 scripts/<script_name>.py` from the project folder.

- **Read positions** disables torque for all configured servo IDs, then reads
  and displays both raw tick positions and angles relative to `ZERO_TICKS` and
  `DIRECTIONS`. Support the arm first so gravity cannot make it drop when
  torque is released.
- **Move all five motors using raw positions (script 02)** asks for five
  integer raw positions from 0 to 4095 in the displayed servo-ID order, then
  sends them together in one synchronized move.
- **Move all five motors using angles (script 03)** asks for five target angles in the
  displayed servo-ID order and shows each joint's configured minimum and
  maximum angle. It converts the values using `ZERO_TICKS` and `DIRECTIONS`,
  validates the configured angle limits, then sends all five targets together.
  Both movement scripts let you press Space during movement to release torque.
- **Kinematics visualization (script 04)** connects to the controller, then
  opens the interactive Robot Arm Kinematics Lab webpage in your browser and
  reads the current raw servo positions without sending movement commands.
  The webpage applies the same conversion as script 06:
  `(raw - ZERO_TICKS) * 360 / (DIRECTIONS * TICKS_PER_REVOLUTION)`.
  The resulting angle is used for both the displayed value and the CAD pose.
  The visualized joints and raw readings refresh twice per second. Script 04
  must remain running while the page is in use; press Ctrl+C in its terminal
  to stop the reader and local webpage server. The live webpage includes a
  confirmed **Release all servo torque** button. Support the arm first:
  releasing torque can let it fall. The page displays Pi-calculated angles using
  `ZERO_TICKS`/`DIRECTIONS`; check the live warning because those zeroes have
  not been confirmed against the CAD home pose. The displayed point is the
  J5 wrist reference, not the jaw TCP.
- **Timestamped motion (script 05)** offers recording or playback. Recording
  first releases torque, reads/displays the initial pose, waits for
  confirmation, then asks for a name and description. It continuously records
  all five servo positions (raw ticks, angles relative to `ZERO_TICKS`, and
  timestamps) every 0.1 seconds until you type `q`. The JSON file is
  checkpointed every two seconds and saved again when recording stops.
- **Waypoint sequence (script 06)** offers recording or playback. Recording
  prompts for raw positions or joint angles and accepts ordered waypoints
  entered from notes. Type `play` after the final point to save and play, or
  `q` to save without playing. Playback uses the current angle settings and
  configured `POSE_SEQUENCE_MOVE_SPEED` and
  `POSE_SEQUENCE_MOVE_ACCELERATION`.
- **Record path** releases torque so you can manually pose the arm, then saves
  a waypoint each time you press Enter. It stores servo ticks, converted
  angles, and the estimated gripper XYZ computed using forward kinematics.
  Record at least two waypoints; this creates a named Cartesian path JSON.
  Re-record paths made before switching to the website hardware-middle setup.
  Support the arm before torque is released. Existing motion files with
  recorded steps are not overwritten; choose a new name for another recording.
- **Play path** lets you choose one of those path JSON files. It interpolates
  between the recorded XYZ points and uses inverse kinematics to calculate
  joint targets along the route. Path resolution and speed are configurable in
  `arm_settings.py`. It enables torque when playback starts; press Space to
  release torque and stop the path.
- **Timestamped playback** lists the recorded-motion JSON files under
  `motions/` and lets you choose by number or name. It plays the initial pose
  and then the raw-position samples, using saved timestamps to time playback.
- **Keyboard control** starts continuous jogging with `a`/`d` for yaw,
  `w`/`s` for gripper height, and `f`/`b` to move the gripper forward/backward
  while keeping its height. The jog continues until you press Space; pressing a
  different direction changes the jog.
  Press `o` to open the gripper to its configured minimum angle, or `p` to close
  it to its configured maximum angle. Yaw direction is reversed from the original
  arrow-key mapping. Multi-joint kinematics moves use the SDK's synchronized
  write. Run it in a terminal window; press `q` to quit. Jog rate is set by
  `KEYBOARD_YAW_SPEED_DEG_PER_SECOND`,
  `KEYBOARD_HEIGHT_SPEED_MM_PER_SECOND`, and
  `KEYBOARD_FORWARD_SPEED_MM_PER_SECOND` in `arm_settings.py`. Servo speed and
  acceleration are set by `KEYBOARD_MOVE_SPEED` and
  `KEYBOARD_MOVE_ACCELERATION`. Height and forward/backward movement use
  inverse kinematics and the dimensions in `arm_settings.py`.
- **GUI** reads all positions and lets you enter target joint angles.
  **Open CAD kinematics webpage** serves the simulator from the sibling
  reference folder on a loopback-only local web server and opens it in your
  browser. Its simulation is the default. The webpage's WebSerial controls use
  that folder's separate calibration; do not send hardware poses until its
  calibration has been checked against the physical arm.

**Emergency stop:** the keyboard controller, motion/path playback, and
one-joint move programs use Space to disable torque on all configured servos.
The GUI has a stop button and Space shortcut. This software stop depends on
the serial link and is not a replacement for a physical power cutoff.
Releasing torque can let the arm fall, so support it and keep clear.

The kinematics are a simple model of the arm, not a camera or collision
planner. Confirm dimensions and joint conventions before using height control.
