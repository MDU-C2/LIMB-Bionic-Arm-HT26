# Christian, Majd
"""Draw BLE preview traces and gravity-referenced orientation."""

from __future__ import annotations

import math
import tkinter as tk

AXIS_COLORS = ("#2563eb", "#d97706", "#159669")


SENSOR_COLORS = ("#7c3aed", "#db2777")


def imu_tilt_degrees(values: list[float]) -> tuple[float, float] | None:
    """Estimate gravity-referenced pitch and roll from one accelerometer sample."""
    if len(values) < 3 or not all(math.isfinite(value) for value in values[:3]):
        return None
    accel_x, accel_y, accel_z = values[:3]
    if math.sqrt(accel_x**2 + accel_y**2 + accel_z**2) < 1e-9:
        return None
    pitch = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)))
    roll = math.degrees(math.atan2(accel_y, accel_z))
    return pitch, roll


def relative_angle(value: float, zero: float) -> float:
    """Return the shortest signed angular difference in degrees."""
    return (value - zero + 180.0) % 360.0 - 180.0


def _draw_traces(
    canvas: tk.Canvas,
    traces: list[list[float]],
    *,
    unit: str = "",
    sample_period: float | None = None,
    limits: tuple[float, float] | None = None,
) -> None:
    """Draw recent samples with a common auto-scaled vertical axis."""
    canvas.delete("all")
    width = max(canvas.winfo_width(), 2)
    height = max(canvas.winfo_height(), 2)
    canvas.create_line(0, height / 2, width, height / 2, fill="#d6dce5")
    values = [value for trace in traces for value in trace if math.isfinite(value)]
    if not values:
        canvas.create_text(
            12, height / 2, text="Waiting for samples...", anchor="w", fill="#667085"
        )
        return
    low, high = limits if limits is not None else (min(values), max(values))
    if high - low < 1e-9:
        low -= 1.0
        high += 1.0
    span = high - low
    left, right, top, bottom = 80, max(width - 12, 82), 14, max(height - 26, 16)
    for value, y in ((high, top), (low, bottom)):
        canvas.create_text(
            left - 8, y, text=f"{value:.2f} {unit}", anchor="e", fill="#667085"
        )
    label = "Received samples"
    if sample_period is not None:
        duration = max(len(trace) - 1 for trace in traces) * sample_period
        label = f"{duration * 1000:.0f} ms of received samples"
    canvas.create_text(right, height - 10, text=label, anchor="e", fill="#667085")
    for index, trace in enumerate(traces):
        if len(trace) < 2:
            continue
        points: list[float] = []
        for sample, value in enumerate(trace):
            if not math.isfinite(value):
                continue
            points.extend(
                (
                    left + sample * (right - left) / (len(trace) - 1),
                    top + (high - max(low, min(high, value))) * (bottom - top) / span,
                )
            )
        if len(points) >= 4:
            canvas.create_line(
                *points,
                fill=AXIS_COLORS[index % len(AXIS_COLORS)],
                width=2,
            )


def _draw_orientation(
    canvas: tk.Canvas,
    tilts: dict[int, tuple[float, float]],
) -> None:
    """Draw gravity-referenced arm indicators for available IMUs."""
    canvas.delete("all")
    width = max(canvas.winfo_width(), 2)
    height = max(canvas.winfo_height(), 2)
    center_x = width * 0.44
    center_y = height * 0.46
    radius = max(min(width, height) * 0.27, 25)
    canvas.create_oval(
        center_x - radius,
        center_y - radius,
        center_x + radius,
        center_y + radius,
        outline="#d6dce5",
    )
    canvas.create_line(center_x - radius, center_y, center_x + radius, center_y, fill="#e5e7eb")
    canvas.create_line(center_x, center_y - radius, center_x, center_y + radius, fill="#e5e7eb")
    if not tilts:
        canvas.create_text(center_x, center_y, text="Waiting for IMU...", fill="#667085")
        return

    for index, (sensor, (pitch, roll)) in enumerate(sorted(tilts.items())):
        color = SENSOR_COLORS[index % len(SENSOR_COLORS)]
        roll_rad = math.radians(roll)
        delta_x = radius * math.sin(roll_rad)
        delta_y = -radius * math.cos(roll_rad)
        canvas.create_line(
            center_x - delta_x * 0.65,
            center_y - delta_y * 0.65,
            center_x + delta_x,
            center_y + delta_y,
            fill=color,
            width=7,
            arrow="last",
        )
        pitch_y = 18 + (90.0 - max(-90.0, min(90.0, pitch))) / 180.0 * (height - 52)
        pitch_x = width - 34 - index * 16
        canvas.create_line(pitch_x, 18, pitch_x, height - 34, fill="#d6dce5", width=2)
        canvas.create_oval(
            pitch_x - 5,
            pitch_y - 5,
            pitch_x + 5,
            pitch_y + 5,
            fill=color,
            outline="",
        )
        canvas.create_text(
            8,
            height - 22 - index * 17,
            text=f"IMU {sensor + 1}: pitch {pitch:+.1f}°  roll {roll:+.1f}°",
            anchor="w",
            fill=color,
            font=("Consolas", 9, "bold"),
        )
