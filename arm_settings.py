"""Edit these values to match the robot arm and Waveshare controller."""

# Serial link from Raspberry Pi to the Waveshare controller.
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Servo IDs and joint order. Verify the five readings with script 01.
JOINT_NAMES = ("base_yaw", "shoulder", "elbow", "wrist", "gripper")
SERVO_IDS = {
    "base_yaw": 1,
    "shoulder": 2,
    "elbow": 3,
    "wrist": 4,
    "gripper": 5,
}

# Set each servo hardware middle using the controller calibration tools first.
# ZERO_TICKS is the raw position the Pi treats as 0 degrees; replace the defaults
# with the measured zero readings for your arm.
TICKS_PER_REVOLUTION = 4096
ZERO_TICKS = {
    "base_yaw": 2048,
    "shoulder": 2048,
    "elbow": 2048,
    "wrist": 2048,
    "gripper": 2048,
}
DIRECTIONS = {
    "base_yaw": 1,
    "shoulder": 1,
    "elbow": 1,
    "wrist": 1,
    "gripper": -1,
}
# Approximate software safety limits relative to the configured middle position.
ANGLE_LIMITS = {
    "base_yaw": (-90.0, 90.0),
    "shoulder": (-90.0, 90.0),
    "elbow": (-135.0, 135.0),
    "wrist": (-135.0, 135.0),
    "gripper": (-90.0, 90.0),
}

MOVE_SPEED = 3000
MOVE_ACCELERATION = 254
POSE_SEQUENCE_MOVE_SPEED = 3000
POSE_SEQUENCE_MOVE_ACCELERATION = 254
TIMESTAMP_MOVE_SPEED = 800
TIMESTAMP_MOVE_ACCELERATION = 50
RECORD_INTERVAL_SECONDS = 0.1
