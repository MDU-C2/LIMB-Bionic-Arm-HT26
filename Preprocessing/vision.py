from __future__ import annotations

from dataclasses import dataclass
import math

from filters import EMARealtime, Point3DProcessor, CameraFilterConfig
from sensor_state import ArmPoseState, HandPoseState, CameraState

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
ARM_LANDMARK_IDS = {"shoulder": 11, "elbow": 13, "wrist": 15}  # Left arm, MediaPipe Pose

@dataclass
class VisionConfig:
    position_alpha:      float = 0.2
    finger_alpha:        float = 0.5
    minimum_visibility:  float = 0.5
    max_position_jump_m: float = 0.25
    max_tracking_gap_s:  float = 0.5

class VisionProcessor:
    """Process one camera observation at a time, online or from a recording.
    Input:  pose.json entries from `data` and `observations`.
    Output: CameraState, ready for the shared SensorState.
    Positions use the recorded `keypoints_camera_m` frame, unchanged.
    No camera/IMU coordinate alignment or clock correction is done here.
    """

    def __init__(self, config: VisionConfig | None = None):
        self.config = config or VisionConfig()
        point_cfg = CameraFilterConfig(
            smoothing_alpha=self.config.position_alpha,
            minimum_visibility=self.config.minimum_visibility,
            max_position_jump_m=self.config.max_position_jump_m,
        )
        self.points = {
            name: Point3DProcessor(point_cfg)
            for name in ARM_LANDMARK_IDS
        }
        self.finger_filters = {
            name: EMARealtime(self.config.finger_alpha)
            for name in FINGERS
        }
        self.grip_reference: dict[str, tuple[float, float]] | None = None
        self.previous_timestamp_ns: int | None = None
        self.previous_grip: tuple[int, float] | None = None

    def calibrate_grip(
        self,
        open_curl: dict[str, float],
        closed_curl: dict[str, float],
    ) -> None:
        """Set measured open/closed finger-curl references (degrees).
        Use the same finger names as `finger_curl_deg` in pose.json.
        Do not call this until the reference capture is verified.
        """
        reference = {}
        for name in ("index", "middle", "ring", "pinky"):
            opened = float(open_curl[name])
            closed = float(closed_curl[name])
            if not (math.isfinite(opened) and math.isfinite(closed)):
                raise ValueError(f"Non-finite grip reference for {name}")
            if closed <= opened:
                raise ValueError(f"Closed reference must exceed open for {name}")
            reference[name] = (opened, closed)
        self.grip_reference = reference
        self.previous_grip = None

    def reset(self) -> None:
        for processor in self.points.values():
            processor.reset()
        for filt in self.finger_filters.values():
            filt.reset()
        self.previous_timestamp_ns = None
        self.previous_grip = None
        # Calibration references intentionally survive a filter reset.

    @staticmethod
    def _point(value: object) -> tuple[float, float, float] | None:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            return None
        try:
            point = tuple(float(v) for v in value)
        except (ValueError, TypeError):
            return None
        if not all(math.isfinite(v) for v in point):
            return None
        return point

    @staticmethod
    def _visibility(obs: dict, landmark_id: int) -> float:
        landmarks = obs.get("pose_landmarks")
        if not isinstance(landmarks, list) or len(landmarks) <= landmark_id:
            return 0.0  # Unknown confidence must not count as visible.
        entry = landmarks[landmark_id]
        try:
            value = float(entry[3])
        except (IndexError, TypeError, ValueError):
            return 0.0
        return value if math.isfinite(value) else 0.0

    def update(self, pose: dict, observation: dict) -> CameraState:
        """`pose` is one entry in data; `observation` the matching entry.

        Both must refer to the same camera frame.
        """
        timestamp_ns = int(observation["host_monotonic_ns"])
        if int(pose["host_monotonic_ns"]) != timestamp_ns:
            raise ValueError("Pose and observation timestamps do not match")

        if self.previous_timestamp_ns is not None:
            dt = (timestamp_ns - self.previous_timestamp_ns) * 1e-9
            if dt <= 0:
                raise ValueError("Vision timestamps must be increasing")
            if dt > self.config.max_tracking_gap_s:
                self.reset()
        self.previous_timestamp_ns = timestamp_ns

        # ---- Arm: stereo-depth-derived positions, in metres ----
        points = pose.get("keypoints_camera_m") or {}
        arm_points = {}
        arm_visible = bool(observation.get("arm_visible", False))
        depth_valid = bool(observation.get("depth_valid", False))

        for name, landmark_id in ARM_LANDMARK_IDS.items():
            raw = self._point(points.get(f"left_{name}"))
            visible = self._visibility(observation, landmark_id)
            arm_points[name] = self.points[name].update(
                point=raw if arm_visible else None,
                visibility=visible,
                depth_valid=depth_valid,
            )

        arm_valid = all(v is not None for v in arm_points.values())
        angles = pose.get("angles_deg") or {}
        shoulder_angles = {}
        for name in ("shoulder_flexion", "shoulder_abduction", "shoulder_rotation_proxy"):
            if arm_valid and angles.get(name) is not None:
                value = float(angles[name])
                if math.isfinite(value):
                    shoulder_angles[name] = value
        elbow_angle = None
        if arm_valid and angles.get("elbow_flexion") is not None:
            value = float(angles["elbow_flexion"])
            if math.isfinite(value):
                elbow_angle = value

        arm = ArmPoseState(
            shoulder=arm_points["shoulder"],
            elbow=arm_points["elbow"],
            wrist=arm_points["wrist"],
            elbow_angle=elbow_angle,
            shoulder_angles=shoulder_angles,
            valid=arm_valid,
        )

        # ---- Hand: existing finger-curl estimates, in degrees ----
        hand = HandPoseState()
        raw_curl = observation.get("finger_curl_deg") or {}
        hand_visible = bool(observation.get("hand_visible", False))
        if hand_visible and isinstance(raw_curl, dict):
            for name in FINGERS:
                if name not in raw_curl:
                    continue
                try:
                    value = float(raw_curl[name])
                except (ValueError, TypeError):
                    continue
                if math.isfinite(value):
                    hand.finger_curl[name] = self.finger_filters[name].update(value)

        # Require all four non-thumb fingers for a usable grip estimate.
        required = ("index", "middle", "ring", "pinky")
        hand.tracking_valid = hand_visible and all(
            name in hand.finger_curl for name in required
        )
        if not hand.tracking_valid:
            self.previous_grip = None
            for filt in self.finger_filters.values():
                filt.reset()  # Do not smooth across an occlusion.
            hand.finger_curl.clear()
        elif self.grip_reference is not None:
            fractions = []
            for name in required:
                opened, closed = self.grip_reference[name]
                fraction = (hand.finger_curl[name] - opened) / (closed - opened)
                fractions.append(max(0.0, min(1.0, fraction)))
            hand.grip_closure = sum(fractions) / len(fractions)
            if self.previous_grip is not None:
                old_t, old_grip = self.previous_grip
                dt = (timestamp_ns - old_t) * 1e-9
                if 0 < dt <= self.config.max_tracking_gap_s:
                    hand.grip_velocity = (hand.grip_closure - old_grip) / dt
            self.previous_grip = (timestamp_ns, hand.grip_closure)

        # No calibrated per-frame tracking confidence is exposed in this format.
        hand.confidence = None
        return CameraState(timestamp_ns=timestamp_ns, arm=arm, hand=hand)

    
    def process_recording(self, recording: dict) -> list[CameraState]:
        """Match pose and observations using camera timestamps."""

        poses = recording.get("data", [])
        observations = recording.get("observations", [])

        self.reset()

        # Index observations by camera timestamp
        observations_by_time = {
            int(obs["host_monotonic_ns"]): obs
            for obs in observations
            if obs.get("host_monotonic_ns") is not None
        }

        states = []
        unmatched = 0

        # Process matched observations chronologically
        for pose in sorted(
            poses,
            key=lambda item: int(item["host_monotonic_ns"])
        ):
            timestamp = int(pose["host_monotonic_ns"])

            observation = observations_by_time.get(timestamp)

            if observation is None:
                unmatched += 1
                continue

            state = self.update(pose, observation)
            states.append(state)

        print(
            f"Vision: {len(poses)} pose frames, "
            f"{len(observations)} observations, "
            f"{len(states)} matched, "
            f"{unmatched} unmatched pose frames"
        )

        return states
