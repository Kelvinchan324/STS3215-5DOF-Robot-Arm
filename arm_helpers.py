"""Small helpers shared by the standalone task programs."""

from pathlib import Path
import sys
import time

PROJECT_FOLDER = Path(__file__).resolve().parent
if str(PROJECT_FOLDER) not in sys.path:
    sys.path.insert(0, str(PROJECT_FOLDER))

from scservo_sdk import COMM_SUCCESS, SMS_STS_TORQUE_ENABLE, PortHandler, sms_sts

import arm_settings as settings


def connect():
    """Open the servo serial port and return its SDK packet handler."""
    port = PortHandler(settings.SERIAL_PORT)
    if not port.openPort():
        raise RuntimeError(f"Could not open serial port {settings.SERIAL_PORT}")
    if not port.setBaudRate(settings.BAUD_RATE):
        port.closePort()
        raise RuntimeError(f"Could not set baud rate to {settings.BAUD_RATE}")
    return port, sms_sts(port)


def check_result(packet_handler, result, error, action):
    if result != COMM_SUCCESS:
        raise RuntimeError(f"{action}: {packet_handler.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"{action}: {packet_handler.getRxPacketError(error)}")


def disable_all_torque(packet_handler):
    """Release torque on every configured servo, reporting any failures."""
    failures = []
    for name in settings.JOINT_NAMES:
        servo_id = settings.SERVO_IDS[name]
        result, error = packet_handler.write1ByteTxRx(
            servo_id, SMS_STS_TORQUE_ENABLE, 0
        )
        try:
            check_result(packet_handler, result, error, f"Disable torque for {name}")
        except RuntimeError as failure:
            failures.append(str(failure))
    if failures:
        raise RuntimeError("Emergency stop incomplete: " + "; ".join(failures))


def enable_all_torque(packet_handler):
    """Enable torque on every configured servo, reporting any failures."""
    failures = []
    for name in settings.JOINT_NAMES:
        servo_id = settings.SERVO_IDS[name]
        result, error = packet_handler.write1ByteTxRx(
            servo_id, SMS_STS_TORQUE_ENABLE, 1
        )
        try:
            check_result(packet_handler, result, error, f"Enable torque for {name}")
        except RuntimeError as failure:
            failures.append(str(failure))
    if failures:
        raise RuntimeError("Could not enable all servos: " + "; ".join(failures))


def wait_for_servo_or_estop(packet_handler, servo_id, terminal):
    """Wait for one move to finish, or release all torque when Space is pressed."""
    return wait_for_servos_or_estop(packet_handler, (servo_id,), terminal)


def wait_for_servos_or_estop(packet_handler, servo_ids, terminal):
    """Wait until every move finishes, or release torque when Space is pressed."""
    while True:
        key = terminal.read_key(0.02)
        if key == " ":
            print("\nEMERGENCY STOP: releasing torque on all servos.", flush=True)
            disable_all_torque(packet_handler)
            return False

        moving_servos = []
        for servo_id in servo_ids:
            moving, result, error = packet_handler.ReadMoving(servo_id)
            check_result(
                packet_handler,
                result,
                error,
                f"Read moving status for servo {servo_id}",
            )
            if moving:
                moving_servos.append(servo_id)
        if not moving_servos:
            return True
        time.sleep(0.02)


def read_servo(packet_handler, servo_id):
    position, result, error = packet_handler.ReadPos(servo_id)
    check_result(packet_handler, result, error, f"Read servo {servo_id}")
    return position


def read_all(packet_handler):
    return {
        name: read_servo(packet_handler, settings.SERVO_IDS[name])
        for name in settings.JOINT_NAMES
    }


def joint_name_for_id(servo_id):
    """Return the configured joint name for a servo ID, or reject unknown IDs."""
    for name, configured_id in settings.SERVO_IDS.items():
        if configured_id == servo_id:
            return name
    valid_ids = ", ".join(str(value) for value in settings.SERVO_IDS.values())
    raise ValueError(f"Servo ID {servo_id} is not configured; use one of: {valid_ids}")


def move_servo(packet_handler, servo_id, position, speed=None, acceleration=None):
    if not 0 <= position <= 4095:
        raise ValueError("Position must be between 0 and 4095")
    if speed is None:
        speed = settings.MOVE_SPEED
    if acceleration is None:
        acceleration = settings.MOVE_ACCELERATION
    result, error = packet_handler.WritePosEx(
        servo_id, position, speed, acceleration
    )
    check_result(packet_handler, result, error, f"Move servo {servo_id}")


def move_all(packet_handler, positions, speed=None, acceleration=None):
    """Send one position command to each named joint in settings order."""
    for name in settings.JOINT_NAMES:
        move_servo(
            packet_handler,
            settings.SERVO_IDS[name],
            int(positions[name]),
            speed=speed,
            acceleration=acceleration,
        )


def move_all_sync(packet_handler, positions, speed=None, acceleration=None):
    """Send selected servo targets in one synchronized SDK packet."""
    move_positions_sync(
        packet_handler,
        positions,
        settings.JOINT_NAMES,
        speed=speed,
        acceleration=acceleration,
    )


def move_positions_sync(
    packet_handler, positions, joint_names, speed=None, acceleration=None
):
    """Send named servo targets in one synchronized SDK packet."""
    if speed is None:
        speed = settings.MOVE_SPEED
    if acceleration is None:
        acceleration = settings.MOVE_ACCELERATION
    try:
        for name in joint_names:
            position = int(positions[name])
            if not 0 <= position <= 4095:
                raise ValueError(f"{name} position must be between 0 and 4095")
            added = packet_handler.SyncWritePosEx(
                settings.SERVO_IDS[name],
                position,
                speed,
                acceleration,
            )
            if not added:
                raise RuntimeError(f"Could not add {name} to synchronized move")
        result = packet_handler.groupSyncWrite.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(packet_handler.getTxRxResult(result))
    finally:
        packet_handler.groupSyncWrite.clearParam()


def angle_to_position(name, angle_degrees):
    if name not in settings.SERVO_IDS:
        raise ValueError(f"Unknown joint name: {name}")
    minimum, maximum = settings.ANGLE_LIMITS[name]
    if not minimum <= angle_degrees <= maximum:
        raise ValueError(
            f"{name} angle {angle_degrees:.2f} is outside the configured "
            f"limits [{minimum:.2f}, {maximum:.2f}] degrees"
        )
    position = round(
        settings.ZERO_TICKS[name]
        + settings.DIRECTIONS[name]
        * angle_degrees
        * settings.TICKS_PER_REVOLUTION
        / 360.0
    )
    if not 0 <= position <= 4095:
        raise ValueError(
            f"{name} angle maps to raw position {position}, outside 0-4095"
        )
    return position


def position_to_angle(name, position):
    if name not in settings.SERVO_IDS:
        raise ValueError(f"Unknown joint name: {name}")
    if not 0 <= position <= 4095:
        raise ValueError(f"{name}: raw position must be between 0 and 4095")
    return (
        (position - settings.ZERO_TICKS[name])
        * 360.0
        / (settings.DIRECTIONS[name] * settings.TICKS_PER_REVOLUTION)
    )


def angles_to_positions(angles):
    return {name: angle_to_position(name, angles[name]) for name in settings.JOINT_NAMES}


def positions_to_angles(positions):
    return {
        name: position_to_angle(name, positions[name])
        for name in settings.JOINT_NAMES
    }
