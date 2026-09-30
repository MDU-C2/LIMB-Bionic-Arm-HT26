"""Normalize AURORA EMG samples and estimate a live muscle activation envelope."""

from __future__ import annotations

import math
from typing import Any


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

    The first half-second at 100 Hz establishes the sensor's DC midpoint and
    resting noise. The peak scale then follows actual contractions, so the
    simulator does not depend on an invented person-specific ADC threshold.
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

    @property
    def calibrated(self) -> bool:
        return self.baseline is not None

    @property
    def calibration_progress(self) -> float:
        return min(1.0, len(self._calibration) / self.calibration_samples)

    def update(self, raw: int | float, dt: float = 0.01) -> float | None:
        value = float(raw)
        if not math.isfinite(value) or not ADC_MIN <= value <= ADC_MAX:
            return None

        if self.baseline is None:
            self._calibration.append(value)
            if len(self._calibration) < self.calibration_samples:
                self.activation = None
                return None
            mean = sum(self._calibration) / len(self._calibration)
            centered = [abs(sample - mean) for sample in self._calibration]
            self.baseline = mean
            self.rest_envelope = sum(centered) / len(centered)
            self.envelope = self.rest_envelope
            self.peak = max(1.0, self.rest_envelope * 4.0)
            self.activation = 0.0
            return self.activation

        dt = max(0.001, min(float(dt), 0.25))
        baseline_alpha = 1.0 - math.exp(-dt / 2.0)
        envelope_alpha = 1.0 - math.exp(-dt / 0.08)
        self.baseline += baseline_alpha * (value - self.baseline)
        rectified = abs(value - self.baseline)
        self.envelope += envelope_alpha * (rectified - self.envelope)

        threshold = max(1.0, self.rest_envelope * 3.0)
        signal = max(0.0, self.envelope - threshold)
        peak_decay = math.exp(-dt / 8.0)
        self.peak = max(signal, self.peak * peak_decay, threshold)
        span = max(self.peak, threshold)
        self.activation = max(0.0, min(1.0, signal / span))
        return self.activation


def select_grip_source(
    emg_activation: float | None,
    camera_hand_curl: float | None,
) -> tuple[float | None, str]:
    """Prefer measured muscle activation, falling back to camera hand curl."""
    if emg_activation is not None:
        return max(0.0, min(1.0, float(emg_activation))), "EMG"
    if camera_hand_curl is not None:
        return max(0.0, min(1.0, float(camera_hand_curl))), "camera"
    return None, "--"
