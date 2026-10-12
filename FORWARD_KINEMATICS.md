# Current motor positions to gripper XYZ

The geometry and encoder home positions in `kinematics_config.json` are copied
from `robot-arm-kinematics-raspberry-pi-2026-10-09/kinematics_config.json`.
This reader uses that calibration independently of the existing movement
programs' `ZERO_TICKS` and `DIRECTIONS`. Confirm these saved home counts still
correspond to the physical CAD home pose, particularly if motors were recentered.
The reference identifies ID 5 as wrist roll; confirm that this matches your arm.

For joint i, with encoder reading r_i, home reading h_i and direction d_i:

```text
q_i = d_i (r_i - h_i) 2π/4096 + zero_offset_rad_i
R_i = I + sin(q_i)[w_i]× + (1 - cos(q_i))[w_i]×²
A_i = [ R_i   (I - R_i)p_i ]
      [ 0 0 0       1       ]
[p_CAD; 1] = A_1 A_2 A_3 A_4 A_5 [p_TCP_home; 1]
```

Here p_i and normalized w_i are each joint's CAD home axis point and direction.
The home encoder counts are `[2042, 3164, 1062, 1957, 2049]`; all reference
directions are +1. No encoder wrapping or limit clamping is applied to readings.

The default output frame puts the origin at the J1 CAD axis reference point
b = `[-17.7264501, -35.4651105, -27.9920143]` mm:

```text
x = p_CAD.x - b.x
y = -(p_CAD.z - b.z)
z = p_CAD.y - b.y
```

Thus z points upward and the axes form a right-handed frame. `--frame cad`
returns the original absolute CAD coordinates (CAD Y is upward).

Read once, or continuously, using the serial settings in `arm_settings.py`:

```sh
python3 scripts/11_read_gripper_xyz.py
python3 scripts/11_read_gripper_xyz.py --watch 0.5
```

Each line is JSON with the point label, frame, units, XYZ, joint angles in radians,
and raw readings. Reads are sequential, so a moving arm's readings may span
slightly different times. This script sends no movement or torque commands.

The source package has **no measured gripper centre point**. Until you add one,
output is labelled `wrist_reference`, representing the J5 axis reference point.
At the saved home readings it is `(-99.8543514, -97.3270974, 234.279446)` mm in
the base frame. It is not the gripper tip.

For gripper coordinates, add `"home_point_mm": [X, Y, Z]` inside the config's
`tcp` object, or pass `--tcp-home X Y Z`. These must be the absolute CAD
coordinates of the jaw centre in the CAD home pose, at a defined jaw opening.
The point is assumed rigidly attached after J5. A changing jaw opening needs
its own mechanism model if it changes the selected point.

Offline check without connecting hardware:

```sh
python3 scripts/11_read_gripper_xyz.py --raw 2042 3164 1062 1957 2049
```

Python API: `position_from_raw({1: r1, 2: r2, 3: r3, 4: r4, 5: r5}, config,
tcp_home_mm=(X, Y, Z))` in `cad_forward_kinematics.py`.
