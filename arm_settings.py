"""Edit these values to match the robot arm and Waveshare controller."""

# Serial link from Raspberry Pi to the Waveshare controller.
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Servo IDs and joint order. Verify IDs with the SDK ping example.
JOINT_NAMES = ("base_yaw", "shoulder", "elbow", "wrist", "gripper")
SERVO_IDS = {
    "base_yaw": 1,
    "shoulder": 2,
    "elbow": 3,
    "wrist": 4,
    "gripper": 5,
}

# Use the website's Calibration Mode to set each servo's hardware middle first.
# ZERO_TICKS is the raw position that the Pi treats as 0 degrees; update it if
# the website-calibrated middle does not report as 2048 on your controller.
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

# CAD joint-axis geometry in millimetres, copied from the reference STEP model.
# Points and axes are in the CAD home frame (X right, Y up, Z depth).
CAD_JOINT_POINTS_MM = (
    (-17.7264501, -35.4651105, -27.9920143),
    (40.9715946, 29.6348895, 4.2684719),
    (81.7487195, 141.8777478, -11.7869588),
    (-46.7502627, 210.1003195, 40.5088566),
    (-117.5808015, 198.8143355, 69.3350831),
)
CAD_JOINT_AXES = (
    (0.0, 1.0, 0.0),
    (0.3769530071, 0.0, 0.9262323847),
    (0.3769530071, 0.0, 0.9262323847),
    (0.3769530071, 0.0, 0.9262323847),
    (-0.9163070995, -0.1460023805, 0.3729136687),
)
# Keep Cartesian hardware jog/path playback disabled until Pi angle zeroes are
# confirmed to match the imported CAD pose. This does not alter servo mapping.
CAD_ZERO_ALIGNMENT_CONFIRMED = False

MOVE_SPEED = 3000
MOVE_ACCELERATION = 254
POSE_SEQUENCE_MOVE_SPEED = 3000
POSE_SEQUENCE_MOVE_ACCELERATION = 254
KEYBOARD_MOVE_SPEED = 800
KEYBOARD_MOVE_ACCELERATION = 50
YAW_STEP_DEGREES = 2.0
HEIGHT_STEP_MM = 5.0
FORWARD_STEP_MM = 5.0
KEYBOARD_CONTROL_INTERVAL_SECONDS = 0.05
KEYBOARD_YAW_SPEED_DEG_PER_SECOND = 20.0
KEYBOARD_HEIGHT_SPEED_MM_PER_SECOND = 20.0
KEYBOARD_FORWARD_SPEED_MM_PER_SECOND = 20.0
PATH_PLAY_INTERVAL_SECONDS = 0.05
PATH_PLAY_SPEED_MM_PER_SECOND = 20.0
PATH_PLAY_MAX_STEP_MM = 2.0
RECORD_INTERVAL_SECONDS = 0.1
