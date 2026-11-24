
from __future__ import annotations

from dataclasses import dataclass
import math

from sensor_state import EMGState, IMUState


# ============================================================
# Generic Filters
# ============================================================

class EMARealtime:
    """Stateful exponential moving average filter."""

    def __init__(self, alpha: float = 0.05):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")

        self.alpha = alpha
        self.value: float | None = None

    def update(self, sample: float) -> float:
        sample = float(sample)
        if self.value is None:
            self.value = sample
        else:
            self.value += self.alpha * (sample - self.value)

        return self.value

    def reset(self) -> None:
        self.value = None


def ema_filter(values: list[float], alpha: float) -> list[float]:
    """EMA for offline recordings."""
    filt = EMARealtime(alpha)
    return [filt.update(v) for v in values]

# ============================================================
# EMG Preprocessing
# ============================================================

@dataclass
class EMGConfig:
    # Nominal EMG sampling: 4000 Hz
    sampling_rate: float = 4000.0

    # Activity smoothing time constant [seconds]
    activity_tau: float = 0.05

    # Maximum accepted sample interval [seconds]
    max_sample_gap: float = 0.005

class EMGProcessor:
    """
    EMG preprocessing:
        ADC
          -> Baseline removal
          -> Absolute activity estimation
          -> EMA smoothing
          -> EMGState

    Designed for Grove EMG Detector.
    """

    def __init__(self, config: EMGConfig | None = None):
        self.config = config or EMGConfig()

        self.baseline: float | None = None

        alpha = 1.0 - math.exp(
            -1.0 / (
                self.config.sampling_rate
                * self.config.activity_tau
            )
        )

        self.activity_filter = EMARealtime(alpha)
        self.previous_timestamp_ns: int | None = None

    def calibrate(
        self,
        resting_samples: list[float],
    ) -> None:
        """Estimate baseline from resting EMG samples."""
        if not resting_samples:
            raise ValueError("Resting samples required")

        self.baseline = (
            sum(resting_samples) / len(resting_samples)
        )

        self.activity_filter.reset()
        self.previous_timestamp_ns = None

    def update(
        self,
        timestamp_ns: int,
        raw_adc: float,
    ) -> EMGState:
        timestamp_ns = int(timestamp_ns)
        raw_adc = float(raw_adc)

        valid = math.isfinite(raw_adc)

        if self.previous_timestamp_ns is not None:
            dt = (
                timestamp_ns - self.previous_timestamp_ns
            ) * 1e-9

            if dt <= 0 or dt > self.config.max_sample_gap:
                valid = False
                self.activity_filter.reset()

        self.previous_timestamp_ns = timestamp_ns

        if not valid or self.baseline is None:
            return EMGState(
                timestamp_ns=timestamp_ns,
                valid=False,
                calibrated=self.baseline is not None,
            )

        # 1. Baseline removal
        centered = raw_adc - self.baseline

        # 2. Activity estimation
        activity_raw = abs(centered)

        # 3. EMA smoothing
        activity = self.activity_filter.update(
            activity_raw
        )

        return EMGState(
            timestamp_ns=timestamp_ns,
            activity=activity,
            valid=True,
            calibrated=True,
        )


# ============================================================
# IMU Preprocessing
# ============================================================

@dataclass
class IMUConfig:
    # EMA smoothing
    smoothing_alpha: float = 0.20

    # Reset filters after large timing gaps [seconds]
    max_sample_gap: float = 0.2

    # 6-axis orientation estimation
    orientation_kp: float = 1.5

    # Accelerometer magnitude limits [g]
    gravity_min_g: float = 0.8
    gravity_max_g: float = 1.2

class IMUProcessor:
    """
    IMU preprocessing:
        Acceleration + Gyroscope
          -> Gyroscope bias correction
          -> EMA smoothing
          -> 6-axis orientation estimation
          -> IMUState

    Input:
        Acceleration [g]
        Gyroscope [deg/s]

    Orientation:
        Quaternion (w, x, y, z)
        Sensor frame -> gravity-aligned reference frame
        Arbitrary heading (yaw not observable)
    """

    def __init__(self, config: IMUConfig | None = None):
        self.config = config or IMUConfig()

        if self.config.orientation_kp < 0:
            raise ValueError("orientation_kp must be >= 0")

        if not (
            0 < self.config.gravity_min_g
            < self.config.gravity_max_g
        ):
            raise ValueError("Invalid gravity magnitude limits")

        self.gyro_bias = (0.0, 0.0, 0.0)
        self.calibrated = False

        self.filters = {
            name: EMARealtime(self.config.smoothing_alpha)
            for name in (
                "ax", "ay", "az",
                "gx", "gy", "gz"
            )
        }

        self.previous_timestamp_ns: int | None = None
        self.orientation: tuple[float, float, float, float] | None = None

    def calibrate(
        self,
        gyro_samples: list[tuple[float, float, float]],
    ) -> None:
        """Estimate gyro bias while the IMU is stationary."""
        if not gyro_samples:
            raise ValueError("Gyro samples required")

        n = len(gyro_samples)

        self.gyro_bias = tuple(
            sum(sample[i] for sample in gyro_samples) / n
            for i in range(3)
        )

        self.calibrated = True
        self.reset_filters()

    def reset_filters(self) -> None:
        for filt in self.filters.values():
            filt.reset()

        self.previous_timestamp_ns = None
        self.orientation = None

    @staticmethod
    def _normalize_quat(q):
        norm = math.sqrt(sum(v * v for v in q))

        if norm < 1e-12 or not math.isfinite(norm):
            return None

        return tuple(v / norm for v in q)

    def _gravity_direction(self, accel):
        """Return normalized accelerometer vector if near 1 g."""
        magnitude = math.sqrt(sum(v * v for v in accel))

        if not (
            self.config.gravity_min_g
            <= magnitude
            <= self.config.gravity_max_g
        ):
            return None

        return tuple(v / magnitude for v in accel)

    def _initialize_orientation(self, accel):
        """
        Align measured +1g specific force with reference +Z.
        Heading remains arbitrary without a magnetometer.
        """
        a = self._gravity_direction(accel)

        if a is None:
            return None

        ax, ay, az = a

        # Handle upside-down initialization
        if az < -0.999999:
            return (0.0, 1.0, 0.0, 0.0)

        q = (
            1.0 + az,
            ay,
            -ax,
            0.0,
        )

        return self._normalize_quat(q)

    def _update_orientation(self, accel, gyro, dt):
        """Gyro integration with gravity-based correction."""
        if self.orientation is None:
            self.orientation = self._initialize_orientation(accel)
            return

        qw, qx, qy, qz = self.orientation

        # Gyroscope deg/s -> rad/s
        wx, wy, wz = (
            math.radians(v) for v in gyro
        )

        a = self._gravity_direction(accel)

        if a is not None:
            ax, ay, az = a

            # Expected +1g direction in sensor coordinates
            gx = 2.0 * (qx*qz - qw*qy)
            gy = 2.0 * (qy*qz + qw*qx)
            gz = 1.0 - 2.0 * (qx*qx + qy*qy)

            # Cross product: measured x predicted
            ex = ay*gz - az*gy
            ey = az*gx - ax*gz
            ez = ax*gy - ay*gx

            # Proportional gravity correction
            kp = self.config.orientation_kp

            wx += kp * ex
            wy += kp * ey
            wz += kp * ez

        # Quaternion derivative
        dq = (
            -0.5 * (qx*wx + qy*wy + qz*wz),
             0.5 * (qw*wx + qy*wz - qz*wy),
             0.5 * (qw*wy - qx*wz + qz*wx),
             0.5 * (qw*wz + qx*wy - qy*wx),
        )

        updated = tuple(
            self.orientation[i] + dq[i] * dt
            for i in range(4)
        )

        self.orientation = self._normalize_quat(updated)

    def update(
        self,
        timestamp_ns: int,
        accel: tuple[float, float, float],
        gyro: tuple[float, float, float],
        gyro_saturated: bool = False,
    ) -> IMUState:

        timestamp_ns = int(timestamp_ns)
        values = (*accel, *gyro)

        valid = (
            all(math.isfinite(float(v)) for v in values)
            and not gyro_saturated
        )

        dt = None

        if self.previous_timestamp_ns is not None:
            dt = (
                timestamp_ns - self.previous_timestamp_ns
            ) * 1e-9

            if dt <= 0:
                return IMUState(
                    timestamp_ns=timestamp_ns,
                    valid=False,
                    calibrated=self.calibrated,
                )

            if dt > self.config.max_sample_gap:
                self.reset_filters()
                dt = None

        if not valid:
            self.reset_filters()

            return IMUState(
                timestamp_ns=timestamp_ns,
                valid=False,
                calibrated=self.calibrated,
            )

        # 1. Gyroscope bias correction
        corrected_gyro = tuple(
            float(gyro[i]) - self.gyro_bias[i]
            for i in range(3)
        )

        # 2. Acceleration EMA
        filtered_accel = tuple(
            self.filters[name].update(value)
            for name, value in zip(
                ("ax", "ay", "az"),
                accel,
            )
        )

        # 3. Gyroscope EMA
        filtered_gyro = tuple(
            self.filters[name].update(value)
            for name, value in zip(
                ("gx", "gy", "gz"),
                corrected_gyro,
            )
        )

        # 4. Orientation estimation
        if self.calibrated:
            self._update_orientation(
                filtered_accel,
                filtered_gyro,
                dt if dt is not None else 0.0,
            )

        self.previous_timestamp_ns = timestamp_ns

        return IMUState(
            timestamp_ns=timestamp_ns,
            accel=filtered_accel,
            gyro=filtered_gyro,
            orientation_quat=self.orientation,
            orientation_valid=self.orientation is not None,
            valid=True,
            calibrated=self.calibrated,
        )

# ============================================================
# Camera - Generic Landmark Filtering
# ============================================================

@dataclass
class CameraFilterConfig:
    smoothing_alpha: float = 0.2
    minimum_visibility: float = 0.5
    max_position_jump_m: float = 0.25


class Point3DProcessor:
    """Filter and validate a single 3D landmark."""
    
    def __init__(
        self,
        config: CameraFilterConfig | None = None,
    ):
        self.rejected_count = 0
        self.config = config or CameraFilterConfig()
        self.filters = [
            EMARealtime(self.config.smoothing_alpha)
            for _ in range(3)
        ]
        self.previous: tuple[float, float, float] | None = None

    def update(
        self,
        point: tuple[float, float, float] | None,
        visibility: float,
        depth_valid: bool,
    ) -> tuple[float, float, float] | None:
        if point is None or not depth_valid:
            return None

        if visibility < self.config.minimum_visibility:
            return None

        if not all(math.isfinite(v) for v in point):
            return None

        if self.previous is not None:
            distance = math.dist(point, self.previous)

            if distance > self.config.max_position_jump_m:
                self.rejected_count += 1

                if self.rejected_count < 3:
                    return None

                self.reset()

        self.rejected_count = 0    
        self.previous = tuple(float(v) for v in point)
        return tuple(
            filt.update(value)
            for filt, value in zip(self.filters, point)
        )

    def reset(self) -> None:
        self.previous = None
        self.rejected_count = 0
        for filt in self.filters:
            filt.reset()
