# Majd
"""Causal raw-EMG filtering; state survives BLE packet boundaries."""

from __future__ import annotations

import math


class Biquad:
    def __init__(self, kind: str, frequency: float, rate: float, q: float) -> None:
        omega = 2 * math.pi * frequency / rate
        cosine = math.cos(omega)
        alpha = math.sin(omega) / (2 * q)
        if kind == "highpass":
            numerator = ((1 + cosine) / 2, -(1 + cosine), (1 + cosine) / 2)
        elif kind == "lowpass":
            numerator = ((1 - cosine) / 2, 1 - cosine, (1 - cosine) / 2)
        else:
            numerator = (1, -2 * cosine, 1)
        denominator = 1 + alpha
        self.b = tuple(value / denominator for value in numerator)
        self.a = (-2 * cosine / denominator, (1 - alpha) / denominator)
        self.kind = kind
        self.initialized = False

    def update(self, value: float) -> float:
        if not self.initialized:
            self.x1 = self.x2 = value
            self.y1 = self.y2 = 0.0 if self.kind == "highpass" else value
            self.initialized = True
        b0, b1, b2 = self.b
        a1, a2 = self.a
        output = b0 * value + b1 * self.x1 + b2 * self.x2 - a1 * self.y1 - a2 * self.y2
        self.x2, self.x1 = self.x1, value
        self.y2, self.y1 = self.y1, output
        return output


class RawEmgFilter:
    """Fourth-order 20 Hz high-pass and 450 Hz low-pass, then a 50 Hz Q=30 notch."""

    def __init__(self, rate: float) -> None:
        self.stages = [
            Biquad(kind, cutoff, rate, q)
            for kind, cutoff in (("highpass", 20), ("lowpass", 450))
            for q in (0.5411961, 1.3065630)
        ] + [Biquad("notch", 50, rate, 30)]

    def update(self, value: float) -> float:
        for stage in self.stages:
            value = stage.update(value)
        return value
