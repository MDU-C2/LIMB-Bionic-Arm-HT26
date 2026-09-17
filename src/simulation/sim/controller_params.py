"""Simulation gains and torque caps, not measured actuator ratings."""

ARM_MOTOR_FORCE_NM = {
    "shoulder_x": 15.0,
    "shoulder_y": 20.0,
    "shoulder_z": 20.0,
    "elbow_x": 10.0,
    "wrist_rotation": 4.0,
}
ARM_POSITION_GAIN = 0.05
ARM_VELOCITY_GAIN = 1.0
FINGER_MOTOR_FORCE_NM = 8.0
FINGER_POSITION_GAIN = 0.6
FINGER_VELOCITY_GAIN = 1.0
FINGER_PREVIEW_SPEED_RAD_S = 4.0
