
from __future__ import annotations
from dataclasses import dataclass, field

# EMG State
@dataclass
class EMGState:
    timestamp_ns: int
    # Muscle activity after preprocessing
    activity: float | None = None
    activity_normalized: float | None = None
    # True = muscle activated
    # False = muscle inactive
    # None = not determined
    activation_state: bool | None = None
    valid: bool = False
    calibrated: bool = False

# IMU State
@dataclass
class IMUState:
    timestamp_ns: int
    # Bias-corrected / filtered acceleration [g]
    accel: tuple[float, float, float] | None = None
    # Bias-corrected / filtered angular velocity [deg/s]
    gyro: tuple[float, float, float] | None = None
    # Orientation quaternion (w, x, y, z)
    # None until orientation estimation is implemented
    orientation_quat: tuple[float, float, float, float] | None = None
    valid: bool = False
    calibrated: bool = False
    orientation_valid: bool = False

# Camera - Arm Pose
@dataclass
class ArmPoseState:
    # 3D positions in camera/world coordinate frame [m]
    shoulder: tuple[float, float, float] | None = None
    elbow: tuple[float, float, float] | None = None
    wrist: tuple[float, float, float] | None = None
    # Joint angles [degrees]
    elbow_angle: float | None = None
    shoulder_angles: dict[str, float] = field(default_factory=dict)
    valid: bool = False

# Camera - Hand Pose
@dataclass
class HandPoseState:
    # Estimated finger flexion [degrees]
    finger_curl: dict[str, float] = field(default_factory=dict)
    # Normalized grip closure:
    # 0.0 = open, 1.0 = closed
    grip_closure: float | None = None
    # Rate of change of grip closure [1/s]
    # Positive = closing, negative = opening
    grip_velocity: float | None = None
    # Hand landmark tracking
    tracking_valid: bool = False
    confidence: float | None = None

# Camera State
@dataclass
class CameraState:
    timestamp_ns: int
    arm: ArmPoseState = field(default_factory=ArmPoseState)
    hand: HandPoseState = field(default_factory=HandPoseState)

# Combined Input for Sensor Fusion
@dataclass
class SensorState:
    timestamp_ns: int
    emg: EMGState | None = None
    upper_arm_imu: IMUState | None = None
    forearm_imu: IMUState | None = None
    camera: CameraState | None = None

# ============================================================
# Future Sensor Fusion Output
# ============================================================
@dataclass
class FusedArmState:
    timestamp_ns: int

    # Fused arm positions [m]
    shoulder: tuple[float, float, float] | None = None
    elbow: tuple[float, float, float] | None = None
    wrist: tuple[float, float, float] | None = None

    # Fused segment orientations (w, x, y, z)
    upper_arm_orientation: tuple[float, float, float, float] | None = None
    forearm_orientation: tuple[float, float, float, float] | None = None

    # Joint angles [degrees]
    elbow_angle: float | None = None

    # Hand state after EMG + Vision fusion
    # OPEN / CLOSING / CLOSED / OPENING / UNKNOWN
    grip_state: str = "UNKNOWN"

    # Continuous grip closure:
    # 0.0 = open, 1.0 = closed
    grip_closure: float | None = None

    # Positive = closing, negative = opening
    grip_velocity: float | None = None

    # Indicates whether fused estimates are usable
    arm_valid: bool = False
    grip_valid: bool = False
