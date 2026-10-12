"""Read-only CAD forward kinematics using the reference encoder calibration."""

import math

from robot_math import _matrix_vector, _rotation_matrix


def position_from_raw(raw_by_id, config, tcp_home_mm=None, frame="base"):
    """Return XYZ mm for a rigid tool point, or the labelled J5 reference.

    raw_by_id maps integer servo IDs to encoder counts. tcp_home_mm is an
    absolute CAD-home point, not an offset from the wrist.
    """
    if frame not in ("base", "cad"):
        raise ValueError("frame must be 'base' or 'cad'")
    joints = config["joints"]
    counts = config["encoder"]["counts_per_revolution"]
    if not math.isfinite(counts) or counts <= 0:
        raise ValueError("counts_per_revolution must be positive and finite")
    if tcp_home_mm is None:
        tcp_home_mm = config.get("tcp", {}).get("home_point_mm")
    label = "gripper_tcp" if tcp_home_mm is not None else "wrist_reference"
    point = tuple(tcp_home_mm if tcp_home_mm is not None else joints[-1]["point_mm"])
    if len(point) != 3 or not all(math.isfinite(v) for v in point):
        raise ValueError("TCP must contain three finite CAD coordinates")
    angles = []
    for joint in joints:
        raw = raw_by_id[joint["servo_id"]]
        if not isinstance(raw, int) or not 0 <= raw < counts:
            raise ValueError(f"Servo {joint['servo_id']}: invalid encoder count {raw}")
        direction = joint["direction"]
        if direction not in (-1, 1):
            raise ValueError("Joint direction must be +1 or -1")
        angle = direction * (raw - joint["home_position"]) * 2 * math.pi / counts
        angle += joint.get("zero_offset_rad", 0.0)
        if not math.isfinite(angle):
            raise ValueError("Joint angle must be finite")
        angles.append(angle)
    # A1 A2 ... A5 p: apply the fixed home-axis transforms right to left.
    for joint, angle in reversed(list(zip(joints, angles))):
        pivot = joint["point_mm"]
        delta = tuple(v - p for v, p in zip(point, pivot))
        rotated = _matrix_vector(_rotation_matrix(joint["axis_unit"], angle), delta)
        point = tuple(v + p for v, p in zip(rotated, pivot))
    if frame == "base":
        x, y, z = (v - p for v, p in zip(point, joints[0]["point_mm"]))
        point = (x, -z, y)
    return {"point": label, "frame": frame, "units": "mm", "xyz": point,
            "angles_rad": angles, "raw_by_id": raw_by_id}
