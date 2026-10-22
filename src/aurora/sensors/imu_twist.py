# Christian, Majd
"""Estimate arm twist from calibrated gyroscope readings."""

from __future__ import annotations

import math

TWIST_AXES = ("x", "y", "z", "-x", "-y", "-z")
DEFAULT_TWIST_AXIS = "y"


class ImuTwistEstimator:
    """Subtract upper-arm motion to find forearm twist.

    The selected axes must run along the mounted arm segments.
    """

    def __init__(
        self,
        shoulder_axis: str = DEFAULT_TWIST_AXIS,
        wrist_axis: str = DEFAULT_TWIST_AXIS,
        deadzone_dps: float = 2.5,
        baseline_s: float = 0.5,
    ) -> None:
        if shoulder_axis not in TWIST_AXES or wrist_axis not in TWIST_AXES:
            raise ValueError("Twist axes must be x, y, z, -x, -y, or -z")
        self.axes = {"shoulder": shoulder_axis, "wrist": wrist_axis}
        self.deadzone_dps = max(0.0, float(deadzone_dps))
        self.baseline_s = max(0.0, float(baseline_s))
        self.reset()

    def reset(self, shoulder_angle: float = 0.0, wrist_angle: float = 0.0) -> None:
        self.bias: dict[str, tuple[float, float, float]] = {}
        self.baseline_time: dict[str, float] = {}
        self.baseline_sum: dict[str, tuple[float, float, float]] = {}
        if not math.isfinite(shoulder_angle) or not math.isfinite(wrist_angle):
            raise ValueError("Twist reference angles must be finite.")
        self.shoulder_home = float(shoulder_angle)
        self.wrist_home = float(wrist_angle)
        self.shoulder_angle = self.shoulder_home
        self.wrist_angle = self.wrist_home
        self.dual_ready = False
        self.camera_aligned = False

    def remap(self, role: str, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        """Express a mounting in the shared bend-X, length-Y, lateral-Z frame."""
        axis = self.axes[role]
        length = "xyz".index(axis[-1])
        bend = 1 if length == 0 else 0
        lateral = 3 - length - bend
        sign = -1.0 if axis.startswith("-") else 1.0
        # Cross(bend, length) keeps this an actual rotation rather than a reflection.
        lateral_sign = 1.0 if (bend, length) in ((0, 1), (1, 2), (2, 0)) else -1.0
        return vector[bend], sign * vector[length], sign * lateral_sign * vector[lateral]

    def update(
        self, gyros: dict[str, tuple[float, float, float]], dt: float, bend_rad: float = 0.0
    ) -> dict[str, float]:
        """Use degrees/s and elapsed sample time; hold still for the rest baseline."""
        if not math.isfinite(dt) or dt <= 0 or dt > 0.5:
            self.reset()
            return {}
        rates: dict[str, tuple[float, float, float]] = {}
        for role in self.axes:
            gyro = gyros.get(role)
            if gyro is None:
                self.bias.pop(role, None)
                self.baseline_time.pop(role, None)
                self.baseline_sum.pop(role, None)
                if role == "shoulder":
                    self.shoulder_angle = self.shoulder_home
                    self.camera_aligned = False
                continue
            if role not in self.bias:
                # Moving during calibration must not become the resting gyro bias.
                if max(abs(value) for value in gyro) > 5.0:
                    self.baseline_time[role] = 0.0
                    self.baseline_sum[role] = (0.0, 0.0, 0.0)
                    continue
                duration = self.baseline_time.get(role, 0.0) + dt
                previous = self.baseline_sum.get(role, (0.0, 0.0, 0.0))
                total = tuple(old + value * dt for old, value in zip(previous, gyro, strict=True))
                self.baseline_time[role], self.baseline_sum[role] = duration, total
                if duration + 1e-9 >= self.baseline_s:
                    self.bias[role] = tuple(value / duration for value in total)
                continue
            rates[role] = tuple(
                value - bias for value, bias in zip(gyro, self.bias[role], strict=True)
            )

        result = {}
        if "shoulder" in self.bias:
            shoulder_rate = rates.get("shoulder", (0.0, 0.0, 0.0))[1]
            if abs(shoulder_rate) <= self.deadzone_dps:
                shoulder_rate = 0.0
            self.shoulder_angle += shoulder_rate * dt
            # The camera's medial-positive proxy has the opposite left-CAD sign.
            result["shoulder_rotation_proxy"] = -self.shoulder_angle
        dual_ready = all(role in self.bias for role in self.axes)
        if dual_ready:
            if self.dual_ready:
                upper_gyro = rates.get("shoulder", (0.0, 0.0, 0.0))
                forearm_gyro = rates.get("wrist", (0.0, 0.0, 0.0))
                # Express shared motion along the forearm's axis at the measured elbow bend.
                # Subtracting raw Y rates would create wrist motion at a bent elbow.
                shared_rate = upper_gyro[1] * math.cos(bend_rad) + upper_gyro[2] * math.sin(
                    bend_rad
                )
                relative_rate = forearm_gyro[1] - shared_rate
                if abs(relative_rate) <= self.deadzone_dps:
                    relative_rate = 0.0
                self.wrist_angle += relative_rate * dt
            else:
                self.wrist_angle = self.wrist_home
            result["wrist_rotation"] = self.wrist_angle
        self.dual_ready = dual_ready
        return result

    def correct_shoulder(self, camera_proxy: float, weight: float) -> bool:
        """Correct the integrated state once per camera observation, without moving wrist."""
        if "shoulder" not in self.bias or not math.isfinite(camera_proxy):
            return False
        weight = max(0.0, min(1.0, weight))
        if weight <= 0.0:
            return False
        target = -camera_proxy
        if not self.camera_aligned:
            # Match the first observed pose when IMU calibration finishes.
            self.shoulder_angle = target
            self.camera_aligned = True
            return True
        self.shoulder_angle += weight * (target - self.shoulder_angle)
        return True
