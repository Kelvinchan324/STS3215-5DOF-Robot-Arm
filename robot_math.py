"""CAD-axis forward and inverse kinematics for the robot arm."""

import math

import arm_settings as settings

_VECTOR_EPSILON = 1e-12
_IK_MAX_ITERATIONS = 60
_IK_ANGLE_STEP_LIMIT_RAD = 0.35


def _dot(left, right):
    return sum(a * b for a, b in zip(left, right))


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _normalize(vector):
    length = math.sqrt(_dot(vector, vector))
    if length < _VECTOR_EPSILON:
        raise ValueError("CAD kinematics contains a zero-length joint axis")
    return tuple(value / length for value in vector)


def _matrix_vector(matrix, vector):
    return tuple(_dot(row, vector) for row in matrix)


def _matrix_product(left, right):
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def _rotation_matrix(axis, angle):
    x, y, z = _normalize(axis)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine
    return (
        (
            cosine + x * x * one_minus_cosine,
            x * y * one_minus_cosine - z * sine,
            x * z * one_minus_cosine + y * sine,
        ),
        (
            y * x * one_minus_cosine + z * sine,
            cosine + y * y * one_minus_cosine,
            y * z * one_minus_cosine - x * sine,
        ),
        (
            z * x * one_minus_cosine - y * sine,
            z * y * one_minus_cosine + x * sine,
            cosine + z * z * one_minus_cosine,
        ),
    )


def _cad_to_arm_frame(vector):
    """Convert CAD X/Y-up/Z-depth to a right-handed X/Y/Z-up arm frame."""
    x, y, z = vector
    return x, -z, y


def _forward_state(angles):
    rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    translation = (0.0, 0.0, 0.0)

    for joint_name, point, axis in zip(
        settings.JOINT_NAMES,
        settings.CAD_JOINT_POINTS_MM,
        settings.CAD_JOINT_AXES,
    ):
        axis_point = tuple(
            value + offset
            for value, offset in zip(_matrix_vector(rotation, point), translation)
        )
        axis_direction = _normalize(_matrix_vector(rotation, axis))
        joint_rotation = _rotation_matrix(
            axis_direction, math.radians(angles[joint_name])
        )
        rotated_axis_point = _matrix_vector(joint_rotation, axis_point)
        joint_translation = tuple(
            value - rotated
            for value, rotated in zip(axis_point, rotated_axis_point)
        )
        translation = tuple(
            value + offset
            for value, offset in zip(
                _matrix_vector(joint_rotation, translation), joint_translation
            )
        )
        rotation = _matrix_product(joint_rotation, rotation)

    cad_base = settings.CAD_JOINT_POINTS_MM[0]
    cad_reference_point = tuple(
        value + offset
        for value, offset in zip(
            _matrix_vector(rotation, settings.CAD_JOINT_POINTS_MM[-1]),
            translation,
        )
    )
    relative_point = tuple(
        value - origin for value, origin in zip(cad_reference_point, cad_base)
    )
    position = _cad_to_arm_frame(relative_point)
    tool_axis = _cad_to_arm_frame(
        _matrix_vector(rotation, settings.CAD_JOINT_AXES[-1])
    )
    return position, _normalize(tool_axis)


def forward_kinematics(angles):
    """Return the J5 wrist-axis reference point (x, y, z), in millimetres."""
    position, _tool_axis = _forward_state(angles)
    return position


def _pose_components(angles):
    position, tool_axis = _forward_state(angles)
    x, y, z = position
    radial = math.hypot(x, y)
    if radial > _VECTOR_EPSILON:
        radial_direction = (x / radial, y / radial)
    else:
        yaw = math.radians(angles["base_yaw"])
        radial_direction = (math.cos(yaw), math.sin(yaw))
    axis_radial = tool_axis[0] * radial_direction[0] + tool_axis[1] * radial_direction[1]
    pitch = math.atan2(tool_axis[2], axis_radial)
    return radial, z, pitch


def tool_pitch_degrees(angles):
    """Return J5-axis pitch relative to the arm's current horizontal radial direction."""
    return math.degrees(_pose_components(angles)[2])


def _solve_linear_system(matrix, values):
    size = len(values)
    augmented = [list(matrix[row]) + [values[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-10:
            raise ValueError("Kinematics target is near a singular arm pose")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][-1] for row in range(size)]


def _solve_inverse_kinematics(reference_angles, variable_joints, target, measure):
    result = dict(reference_angles)
    target_pitch = target[-1]
    for _iteration in range(_IK_MAX_ITERATIONS):
        measured = measure(result)
        errors = [wanted - actual for wanted, actual in zip(target, measured)]
        errors[-1] = math.atan2(math.sin(errors[-1]), math.cos(errors[-1]))
        if max(abs(error) for error in errors) < 1e-8:
            for joint_name, angle in result.items():
                minimum, maximum = settings.ANGLE_LIMITS[joint_name]
                if not minimum <= angle <= maximum:
                    raise ValueError(
                        f"Movement needs {joint_name}={angle:.2f} degrees, outside "
                        f"the configured limits [{minimum:.2f}, {maximum:.2f}] degrees"
                    )
            return result

        jacobian = [[] for _ in errors]
        delta_rad = 1e-5
        for joint_name in variable_joints:
            original = result[joint_name]
            result[joint_name] = original + math.degrees(delta_rad)
            plus = measure(result)
            result[joint_name] = original - math.degrees(delta_rad)
            minus = measure(result)
            result[joint_name] = original
            for row in range(len(errors)):
                difference = plus[row] - minus[row]
                if row == len(errors) - 1:
                    difference = math.atan2(math.sin(difference), math.cos(difference))
                jacobian[row].append(difference / (2 * delta_rad))

        try:
            updates = _solve_linear_system(jacobian, errors)
        except ValueError:
            # A small damped least-squares step can escape near-singular poses.
            transpose = list(map(list, zip(*jacobian)))
            normal = [
                [
                    sum(transpose[row][k] * jacobian[k][column] for k in range(len(errors)))
                    + (1e-5 if row == column else 0.0)
                    for column in range(len(variable_joints))
                ]
                for row in range(len(variable_joints))
            ]
            projected_error = [
                sum(transpose[row][k] * errors[k] for k in range(len(errors)))
                for row in range(len(variable_joints))
            ]
            updates = _solve_linear_system(normal, projected_error)

        for joint_name, update in zip(variable_joints, updates):
            bounded_update = max(
                -_IK_ANGLE_STEP_LIMIT_RAD,
                min(_IK_ANGLE_STEP_LIMIT_RAD, update),
            )
            result[joint_name] += math.degrees(bounded_update)

    final_measurement = measure(result)
    position_error = max(
        abs(wanted - actual)
        for wanted, actual in zip(target[:-1], final_measurement[:-1])
    )
    pitch_error = math.degrees(
        math.atan2(
            math.sin(target_pitch - final_measurement[-1]),
            math.cos(target_pitch - final_measurement[-1]),
        )
    )
    raise ValueError(
        f"Target is unreachable or outside joint limits "
        f"(remaining position error {position_error:.2f} mm, "
        f"pitch error {pitch_error:.2f} degrees)"
    )


def move_tip_height(angles, height_change_mm):
    """Jog the J5 reference point vertically while preserving radius and pitch."""
    _x, _y, z = forward_kinematics(angles)
    radial, _current_z, pitch = _pose_components(angles)
    target = (radial, z + height_change_mm, pitch)

    def measure(candidate):
        return _pose_components(candidate)

    return _solve_inverse_kinematics(
        angles,
        ("shoulder", "elbow", "wrist"),
        target,
        measure,
    )


def move_tip_forward(angles, distance_mm):
    """Jog the J5 reference point radially, preserving height and pitch."""
    _x, _y, z = forward_kinematics(angles)
    radial, _current_z, pitch = _pose_components(angles)
    target = (radial + distance_mm, z, pitch)

    def measure(candidate):
        return _pose_components(candidate)

    return _solve_inverse_kinematics(
        angles,
        ("shoulder", "elbow", "wrist"),
        target,
        measure,
    )


def inverse_kinematics(x, y, z, tool_pitch_degrees_value, reference_angles):
    """Solve a Cartesian target and J5-axis pitch near a reference pose."""
    target_pitch = math.radians(tool_pitch_degrees_value)
    target = (x, y, z, target_pitch)

    def measure(candidate):
        position, _tool_axis = _forward_state(candidate)
        return (*position, _pose_components(candidate)[2])

    return _solve_inverse_kinematics(
        reference_angles,
        ("base_yaw", "shoulder", "elbow", "wrist"),
        target,
        measure,
    )
