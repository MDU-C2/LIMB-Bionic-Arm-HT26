"""Fixed control/physics steps with explicit wall time and retained backlog."""

from __future__ import annotations

import math


class FixedStepClock:
    def __init__(self, frequency_hz=60, max_steps=12):
        if frequency_hz <= 0 or max_steps < 1:
            raise ValueError("Positive frequency and step budget required")
        self.period_s = 1.0 / frequency_hz
        self.max_steps = max_steps
        self.wall_time_s = 0.0
        self.pending_s = 0.0

    def advance(self, elapsed_s):
        if not math.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError("Elapsed time must be finite and nonnegative")
        self.wall_time_s += elapsed_s
        self.pending_s += elapsed_s
        count = min(self.max_steps, int((self.pending_s + 1e-12) / self.period_s))
        self.pending_s = max(0.0, self.pending_s - count * self.period_s)
        return count
