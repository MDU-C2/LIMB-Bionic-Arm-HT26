# Majd
"""Normalize AURORA EMG samples and estimate a live muscle activation envelope."""

from __future__ import annotations

import math
from typing import Any

from aurora.sensors.emg_filter import RawEmgFilter

ADC_MIN = 0
ADC_MAX = 4095


def extract_emg(packet: object) -> dict[str, Any]:
    """Return the current single-channel EMG sample from a serial packet."""
    if not isinstance(packet, dict):
        return {"connected": False, "error": "missing packet"}
    value = packet.get("emg")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        value = {"adc_raw": value}
    if not isinstance(value, dict):
        return {"connected": False, "error": "missing from packet"}

    raw = value.get("adc_raw", value.get("raw"))
    valid_raw = (
        isinstance(raw, (int, float))
        and not isinstance(raw, bool)
        and math.isfinite(float(raw))
        and ADC_MIN <= float(raw) <= ADC_MAX
    )
    connected = bool(value.get("connected", valid_raw)) and valid_raw
    result: dict[str, Any] = {
        "connected": connected,
        "gpio": value.get("gpio", "?"),
        "adc_channel": value.get("adc_channel", "?"),
    }
    if valid_raw:
        result["adc_raw"] = int(round(float(raw)))
    if not connected:
        result["error"] = str(value.get("error", "no valid ADC sample"))
    return result


class EmgActivationEstimator:
    """Create a rectified, smoothed activation using a measured rest baseline.

    Rest establishes the noise floor. Raw high-rate samples use a causal
    band-pass, mains notch and RMS envelope. A deliberate strong-grip trial
    fixes the range; the provisional range never decays during relaxation.
    """

    def __init__(self, calibration_samples: int = 50) -> None:
        if calibration_samples < 2:
            raise ValueError("calibration_samples must be at least 2")
        self.calibration_samples = calibration_samples
        self.reset()

    def reset(self) -> None:
        self._calibration: list[float] = []
        self.baseline: float | None = None
        self.rest_envelope = 0.0
        self.envelope = 0.0
        self.peak = 1.0
        self.activation: float | None = None
        self._filter = None
        self._period = None
        self._rest_energy: list[float] = []
        self._energy = 0.0
        self._clipping = 0.0
        self.reference: float | None = None
        self._strength_elapsed: float | None = None
        self._strength_samples: list[float] = []
        self._strength_sample_time = 0.0
        self.calibration_error = ""
        self.strong_envelope: float | None = None
        self._gap_settling_s = 0.0

    @property
    def status(self) -> str:
        if self._gap_settling_s > 0:
            return "EMG packet gap: waiting for continuous samples"
        if not self.calibrated:
            return "Relax muscle: measuring rest"
        if self._clipping > 0.05:
            return "ADC clipping: check electrodes/gain"
        if self._strength_elapsed is not None:
            return f"Hold a strong comfortable grip: {max(0, 3 - self._strength_elapsed):.1f} s"
        if self.calibration_error:
            return self.calibration_error
        return (
            "Grip calibrated"
            if self.reference is not None
            else "Calibrate strong grip (scale estimated)"
        )

    @property
    def calibration_state(self) -> dict:
        """Machine state is independent of the instructions shown to the operator."""
        if self._gap_settling_s > 0:
            phase, progress = "gap", 0.0
        elif self._clipping > 0.05:
            phase, progress = "clipping", 0.0
        elif self.calibration_error:
            phase, progress = "failed", 0.0
        elif not self.calibrated:
            phase, progress = "rest", self.calibration_progress
        elif self._strength_elapsed is not None:
            phase, progress = "grip", min(1.0, self._strength_elapsed / 3)
        elif self.reference is not None:
            phase, progress = "ready", 1.0
        else:
            phase, progress = "grip_reference_needed", 0.0
        return {"phase": phase, "progress": progress, "message": self.status}

    def calibrate_strength(self) -> bool:
        """Capture three seconds of deliberate grip after a relaxed baseline."""
        if not self.calibrated:
            return False
        self._strength_elapsed = 0.0
        self._strength_samples = []
        self._strength_sample_time = 0.0
        self.calibration_error = ""
        self.reference = None
        self.strong_envelope = None
        self.activation = None
        return True

    @property
    def activation_threshold(self) -> float:
        """Separate activation from the measured noise, independent of ADC gain."""
        return max(1.0, self.rest_envelope * 1.5)

    @property
    def minimum_grip_envelope(self) -> float:
        return self.activation_threshold + max(1.0, self.rest_envelope * 0.5)

    def sample_gap(self) -> None:
        """Avoid interpreting a discontinuous waveform as a muscle contraction."""
        if self._period is not None and self._filter is not None:
            self._filter = RawEmgFilter(1 / self._period)
            self._gap_settling_s = 0.25
        if not self.calibrated:
            self._calibration.clear()
            self._rest_energy.clear()
        if self._strength_elapsed is not None:
            self._strength_elapsed = None
            self.calibration_error = "Grip calibration interrupted by missing BLE samples; repeat"
        self._energy = self.rest_envelope**2
        self.envelope = self.rest_envelope
        self.activation = None

    @property
    def calibrated(self) -> bool:
        return self.baseline is not None

    @property
    def calibration_progress(self) -> float:
        return min(1.0, len(self._calibration) / self.calibration_samples)

    def update(self, raw: int | float, dt: float = 0.01) -> float | None:
        value = float(raw)
        if not math.isfinite(value) or not ADC_MIN <= value <= ADC_MAX:
            self.activation = None
            return None

        if not math.isfinite(dt) or dt <= 0:
            self.activation = None
            return None
        dt = max(0.000001, min(float(dt), 0.25))
        if self._period is None:
            self._period = dt
            self._filter = RawEmgFilter(1 / dt) if dt <= 0.001 else None
        filtered = self._filter.update(value) if self._filter is not None else None
        self._clipping += (1 - math.exp(-dt / 0.1)) * (
            float(value <= 2 or value >= ADC_MAX - 2) - self._clipping
        )
        if self._gap_settling_s > 0:
            self._gap_settling_s = max(0.0, self._gap_settling_s - dt)
            self.activation = None
            return None

        if self.baseline is None:
            self._calibration.append(value)
            # The causal band-pass/notch needs time to settle after reset.
            # Counting that transient as muscle noise inflated the rest floor.
            settling_samples = min(self.calibration_samples // 2, round(0.25 / dt))
            if filtered is not None and len(self._calibration) > settling_samples:
                self._rest_energy.append(filtered * filtered)
            if len(self._calibration) < self.calibration_samples:
                self.activation = None
                return None
            mean = sum(self._calibration) / len(self._calibration)
            centered = [abs(sample - mean) for sample in self._calibration]
            self.baseline = mean
            self.rest_envelope = (
                math.sqrt(sum(self._rest_energy) / len(self._rest_energy))
                if self._rest_energy
                else sum(centered) / len(centered)
            )
            self.envelope = self.rest_envelope
            self._energy = self.rest_envelope**2
            self.peak = max(50.0, self.rest_envelope * 10.0)
            self.activation = 0.0
            return self.activation

        baseline_alpha = 1.0 - math.exp(-dt / 2.0)
        envelope_alpha = 1.0 - math.exp(-dt / 0.08)
        if filtered is not None:
            self._energy += envelope_alpha * (filtered * filtered - self._energy)
            self.envelope = math.sqrt(max(0.0, self._energy))
        else:
            # Low-rate legacy packets may be a hardware envelope. They cannot
            # support a 450 Hz digital filter. Do not absorb a held contraction.
            if self.activation is not None and self.activation < 0.05:
                self.baseline += baseline_alpha * (value - self.baseline)
            rectified = abs(value - self.baseline)
            self.envelope += envelope_alpha * (rectified - self.envelope)

        threshold = self.activation_threshold
        signal = max(0.0, self.envelope - threshold)
        if self._clipping > 0.05:
            self.calibration_error = "ADC clipping: repeat rest and strong-grip calibration"
            self._strength_elapsed = None
            self.activation = None
            return None
        if self._strength_elapsed is not None:
            self._strength_elapsed += dt
            self._strength_sample_time += dt
            if self._strength_elapsed > 0.3 and self._strength_sample_time >= 0.02:
                self._strength_samples.append(self.envelope)
                self._strength_sample_time = 0.0
            if self._strength_elapsed >= 3:
                samples = sorted(self._strength_samples)
                # Use sustained contraction rather than a brief movement spike.
                strong = samples[len(samples) // 2]
                self.strong_envelope = strong
                if strong <= self.minimum_grip_envelope:
                    self.calibration_error = (
                        f"No clear grip signal: rest {self.rest_envelope:.1f}, "
                        f"grip {strong:.1f}, need >{self.minimum_grip_envelope:.1f} RMS ADC; "
                        "check electrode contact/gain and repeat"
                    )
                else:
                    self.reference = strong - threshold
                self._strength_elapsed = None
            self.activation = None
            return None
        if self.calibration_error:
            self.activation = None
            return None
        self.peak = max(signal, self.peak, threshold)
        span = self.reference if self.reference is not None else self.peak
        self.activation = max(0.0, min(1.0, signal / span))
        return self.activation


def select_grip_source(
    emg_activation: float | None,
    camera_hand_curl: float | None,
    mode: str = "auto",
) -> tuple[float | None, str]:
    """Select grip explicitly; arriving EMG packets do not prove electrode contact."""
    if mode not in ("auto", "camera", "emg"):
        raise ValueError("Grip source must be auto, camera, or emg")
    if mode != "camera" and emg_activation is not None:
        return max(0.0, min(1.0, float(emg_activation))), "EMG"
    if mode != "emg" and camera_hand_curl is not None:
        return max(0.0, min(1.0, float(camera_hand_curl))), "camera"
    return None, "--"
