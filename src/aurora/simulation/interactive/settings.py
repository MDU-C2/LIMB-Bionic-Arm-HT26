# Majd, Christian
"""Interactive simulator arguments, reference poses, and task settings."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from aurora.sensors.imu_twist import DEFAULT_TWIST_AXIS, TWIST_AXES
from aurora.sensors.settings import DEFAULT_BLE_DEVICE_NAME, DEFAULT_GRIP_SOURCE
from aurora.simulation.model.joint_limits import LEFT_ARM_LIMITS_DEG as LEFT_ARM_LIMITS_DEG
from aurora.simulation.model.robot_model import LEFT_GRIP_OFFSET_M


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("kinematic", "dynamic"),
        default="kinematic",
        help="Direct joint preview or gravity-and-motor physics mode",
    )
    parser.add_argument(
        "--telemetry-out",
        type=Path,
        help="Write dynamic-mode state and torque snapshots as JSON lines",
    )
    parser.add_argument(
        "--torque-out", type=Path, help="Write a readable dynamic-mode arm torque CSV"
    )
    parser.add_argument(
        "--control",
        choices=("keyboard", "camera-imu"),
        default="keyboard",
        help="Use keyboard controls or guided OAK-D and dual-IMU BLE control",
    )
    parser.add_argument("--port", help="Dual-IMU/EMG ESP32 serial port for live control")
    parser.add_argument(
        "--transport", choices=("ble", "serial"), help="Live sensors: BLE by default"
    )
    parser.add_argument(
        "--device", default=DEFAULT_BLE_DEVICE_NAME, help="ESP32 BLE advertised name"
    )
    parser.add_argument("--address", default="", help="Optional known BLE address")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--subject", default="diagnostic", help="Subject ID saved with live calibration"
    )
    parser.add_argument("--camera-weight", type=float, default=0.25)
    parser.add_argument(
        "--shoulder-twist-axis",
        choices=TWIST_AXES,
        default=DEFAULT_TWIST_AXIS,
        help="Signed sensor axis along the upper arm; verify after mounting",
    )
    parser.add_argument(
        "--wrist-twist-axis",
        choices=TWIST_AXES,
        default=DEFAULT_TWIST_AXIS,
        help="Signed sensor axis along the forearm; align its sign with the upper arm",
    )
    parser.add_argument(
        "--grip-source",
        choices=("camera", "emg", "auto"),
        default=DEFAULT_GRIP_SOURCE,
        help="EMG grip by default; Camera and Auto are also available",
    )
    parser.add_argument("--depth", action="store_true", help="Use OAK-D stereo depth")
    parser.add_argument("--emg-model", type=Path, help="Explicitly selected, reviewed EMG bundle")
    parser.add_argument("--replay", type=Path, help="Replay a recorded cup trial without hardware")
    parser.add_argument("--table-width-cm", type=float, default=60)
    parser.add_argument("--table-depth-cm", type=float, default=60)
    args = parser.parse_args(argv)
    if args.replay is not None:
        args.control = "camera-imu"
        args.transport = "ble"
    dynamic_mode = args.mode == "dynamic"
    if (args.telemetry_out is not None or args.torque_out is not None) and not dynamic_mode:
        parser.error("--telemetry-out and --torque-out require --mode dynamic")
    args.transport = args.transport or ("serial" if args.port else "ble")
    if args.emg_model is not None and (
        args.control != "camera-imu" or args.transport != "ble" or args.grip_source != "emg"
    ):
        parser.error("--emg-model requires camera-imu control, BLE transport and EMG grip.")
    if args.control == "camera-imu" and args.transport == "serial" and not args.port:
        parser.error("--port is required for serial live control")
    if args.baud <= 0:
        parser.error("--baud must be positive")
    if not 0.0 <= args.camera_weight <= 1.0:
        parser.error("--camera-weight must be between 0 and 1")
    if any(
        not math.isfinite(value) or value < 12
        for value in (
            args.table_width_cm,
            args.table_depth_cm,
        )
    ):
        parser.error("Table width and depth must be finite and at least 12 cm")
    return args


SIMULATION_FREQUENCY_HZ = 60
PHYSICS_SUBSTEPS = 4
TARGET_RADIUS_M = 0.04
TARGET_HEIGHT_M = 0.10
SAFE_REACH_DISTANCE_M = 0.10
# The HUD rounds to two decimals. A little margin avoids showing 0.20 m while
# rejecting the manual F grip because the unrounded distance is just above it.
GRASP_CAPTURE_DISTANCE_M = 0.25
TARGET_NORMAL_MASS_KG = 0.1
TABLE_TOP_HEIGHT_M = 0.425
TABLE_THICKNESS_M = 0.03
TABLE_CENTER_XY = (0.0, 0.58)
# The torso centerline is world X=0. Keep the left shoulder outside the
# tabletop edge, with an inward mounting angle for the pick-and-place task.
ARM_BASE_POSITION_M = (-0.43, 0.0, 0.7)
ARM_BASE_YAW_DEG = -40.0
OVERVIEW_CAMERA_TARGET = (-0.10, 0.35, 0.45)
OVERVIEW_CAMERA_DISTANCE_M = 1.35
OVERVIEW_CAMERA_YAW_DEG = 0
OVERVIEW_CAMERA_PITCH_DEG = -28
HAND_GRIP_OFFSET_M = LEFT_GRIP_OFFSET_M
MIN_GRASP_CURL = 0.45
MAX_GRASP_CURL = 1.5
MIN_GRASP_FINGERTIPS = 2
FINGERTIP_CONTACT_FORCE_N = 0.15
CONTACT_STIFFNESS_N_PER_M = 500.0
AUTO_GRASP_CURL_PER_SECOND = 0.6
AUTO_GRASP_WRIST_DEG = 20.0

FINGER_CURL_PER_SECOND = 3.0
FINGER_JOINT_TOKENS = ("thumb", "index", "middle", "ring", "pinky")

READY_POSE_DEG = {
    "shoulder_x": 0.0,
    "shoulder_y": 45.0,
    "shoulder_z": -35.0,
    "elbow_x": 45.0,
    # Keep the wrist neutral on reset.
    "wrist_rotation": 0.0,
}
LIVE_ALIGNMENT_POSE_DEG = dict(READY_POSE_DEG)
CAMERA_MODES = ("overview", "shoulder", "hand")
