"""Simulation gains and torque caps, not measured actuator ratings."""

ARM_MOTOR_FORCE_NM = {
    "shoulder_x": 15.0,
    "shoulder_y": 20.0,
    "shoulder_z": 20.0,
    "elbow_x": 10.0,
    "wrist_rotation": 4.0,
}

ARM_ACTUATOR_INFO = {
    "shoulder_x": {
        "movement": "upper-arm rotation",
        "model": "NEMA17-04",
        "mapping": "HT25 shoulder firmware",
        "published_rating": "0.45 Nm holding at 1.5 A",
    },
    "shoulder_y": {
        "movement": "shoulder up/down",
        "model": "JX PDI-HV2060MG",
        "mapping": "HT25 shoulder firmware",
        "published_rating": "4.7 Nm stall at 6 V; 6.1 Nm stall at 7.4 V",
    },
    "shoulder_z": {
        "movement": "shoulder left/right",
        "model": "JX PDI-HV2060MG",
        "mapping": "HT25 shoulder firmware",
        "published_rating": "4.7 Nm stall at 6 V; 6.1 Nm stall at 7.4 V",
    },
    "elbow_x": {
        "movement": "elbow bend",
        "model": "NEMA17-04",
        "mapping": "HT25 elbow firmware",
        "published_rating": "0.45 Nm holding at 1.5 A",
    },
    "wrist_rotation": {
        "movement": "lower-arm/wrist rotation",
        "model": "Whadda WPK601",
        "mapping": "HT25 hand firmware",
        "published_rating": "not verified",
    },
}
ARM_POSITION_GAIN = 0.05
ARM_VELOCITY_GAIN = 1.0
FINGER_MOTOR_FORCE_NM = 8.0
FINGER_POSITION_GAIN = 0.6
FINGER_VELOCITY_GAIN = 1.0
FINGER_PREVIEW_SPEED_RAD_S = 4.0
