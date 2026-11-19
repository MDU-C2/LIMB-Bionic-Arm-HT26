
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
    # Optional activation thresholds in normalized units
    threshold_on: float | None = None
    threshold_off: float | None = None


class EMGProcessor:
    """
    EMG preprocessing:
        ADC
          -> Baseline removal
          -> Activity estimation
          -> EMA smoothing
          -> Optional normalization
          -> Optional activation detection
    Designed for Grove EMG Detector.
    """

    def __init__(self, config: EMGConfig | None = None):
        self.config = config or EMGConfig()
        self.baseline: float | None = None
        self.reference: float | None = None
        alpha = 1.0 - math.exp(
            -1.0 / (
                self.config.sampling_rate
                * self.config.activity_tau
            )
        )
        self.activity_filter = EMARealtime(alpha)
        self.activation: bool | None = None
        self.previous_timestamp_ns: int | None = None

    def calibrate(
        self,
        resting_samples: list[float],
        active_samples: list[float] | None = None,
    ) -> None:
        if not resting_samples:
            raise ValueError("Resting samples required")

        self.baseline = sum(resting_samples) / len(resting_samples)
        self.reference = None
        if active_samples:
            # Initial activity reference for normalization.
            deviations = [
                abs(value - self.baseline)
                for value in active_samples
            ]
            ref = sum(deviations) / len(deviations)
            if ref > 0:
                self.reference = ref

        self.activity_filter.reset()
        self.activation = None
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
                self.activation = None

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

        # 3. Smoothing
        activity = self.activity_filter.update(
            activity_raw
        )

        # 4. Optional normalization
        normalized = None
        if self.reference is not None:
            normalized = activity / self.reference

        # 5. Optional threshold + hysteresis
        on = self.config.threshold_on
        off = self.config.threshold_off
        if (
            normalized is not None
            and on is not None
            and off is not None
        ):
            if off >= on:
                raise ValueError(
                    "threshold_off must be < threshold_on"
                )

            if self.activation is None:
                self.activation = normalized >= on
            elif not self.activation and normalized >= on:
                self.activation = True
            elif self.activation and normalized <= off:
                self.activation = False

        return EMGState(
            timestamp_ns=timestamp_ns,
            activity=activity,
            activity_normalized=normalized,
            activation_state=self.activation,
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


class IMUProcessor:
    """
    IMU preprocessing:
        Acceleration + Gyroscope
          -> Gyroscope bias correction
          -> EMA smoothing
          -> IMUState
    Input units:
        Acceleration: g
        Gyroscope: degrees/second
    Orientation estimation will be added separately.
    """

    def __init__(self, config: IMUConfig | None = None):
        self.config = config or IMUConfig()
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

    def calibrate(
        self,
        gyro_samples: list[tuple[float, float, float]],
    ) -> None:
        """Estimate gyro bias while IMU is stationary."""
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

        if self.previous_timestamp_ns is not None:
            dt = (
                timestamp_ns - self.previous_timestamp_ns
            ) * 1e-9
            if dt <= 0:
                valid = False
            elif dt > self.config.max_sample_gap:
                self.reset_filters()

        if not valid:
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

        # 2. Acceleration smoothing
        filtered_accel = tuple(
            self.filters[name].update(value)
            for name, value in zip(
                ("ax", "ay", "az"),
                accel,
            )
        )

        # 3. Gyroscope smoothing
        filtered_gyro = tuple(
            self.filters[name].update(value)
            for name, value in zip(
                ("gx", "gy", "gz"),
                corrected_gyro,
            )
        )
        self.previous_timestamp_ns = timestamp_ns
        return IMUState(
            timestamp_ns=timestamp_ns,
            accel=filtered_accel,
            gyro=filtered_gyro,
            orientation_quat=None,
            orientation_valid=False,
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
