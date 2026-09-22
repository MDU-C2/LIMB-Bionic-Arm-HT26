"""Live history graph for simulated joint motor torque."""

from __future__ import annotations

from collections import deque


class TorqueHistory:
    def __init__(self, joint_names, max_samples: int = 600):
        if max_samples < 2:
            raise ValueError("max_samples must be at least 2")
        self.max_samples = max_samples
        self.values = {
            name: deque(maxlen=max_samples) for name in joint_names
        }

    def append(self, torques) -> None:
        for name, values in self.values.items():
            values.append(float(torques.get(name, 0.0)))

    def points(self, name: str, left: float, right: float,
               top: float, bottom: float, limit: float) -> list[float]:
        values = self.values[name]
        if not values or limit <= 0:
            return []
        width = right - left
        center = (top + bottom) / 2.0
        half_height = (bottom - top) / 2.0
        first_x = right - width * (len(values) - 1) / (self.max_samples - 1)
        step = width / (self.max_samples - 1)
        points: list[float] = []
        for index, value in enumerate(values):
            points.extend((
                first_x + index * step,
                center - max(-limit, min(limit, value)) / limit * half_height,
            ))
        return points


class TorqueGraph:
    COLORS = ("#3b82f6", "#22c55e", "#f59e0b", "#ef4444", "#a855f7")

    def __init__(self, parent, tk_module, joints: dict[str, str],
                 torque_limit_nm: float, sample_rate_hz: int = 60,
                 seconds: int = 10):
        self.joints = joints
        self.limit = float(torque_limit_nm)
        self.seconds = seconds
        self.history = TorqueHistory(joints, sample_rate_hz * seconds)
        self.canvas = tk_module.Canvas(
            parent, width=470, height=205, background="#111827",
            highlightthickness=1, highlightbackground="#374151",
        )
        self.canvas.pack(fill="x", padx=10, pady=(0, 8))

    def append(self, torques) -> None:
        self.history.append(torques)

    def redraw(self, active: bool) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 470)
        height = max(canvas.winfo_height(), 205)
        left, right, top, bottom = 42, width - 10, 30, height - 30

        for fraction in (-1.0, -0.5, 0.0, 0.5, 1.0):
            y = (top + bottom) / 2.0 - fraction * (bottom - top) / 2.0
            canvas.create_line(left, y, right, y,
                               fill="#6b7280" if fraction == 0 else "#263244")
            canvas.create_text(left - 5, y, anchor="e", fill="#9ca3af",
                               font=("Consolas", 8),
                               text=f"{fraction * self.limit:.0f}")
        canvas.create_text(left, 10, anchor="w", fill="#d1d5db",
                           font=("Segoe UI", 9),
                           text=f"PyBullet motor torque (Nm), last {self.seconds} s")

        legend_x = left
        for color, (name, label) in zip(self.COLORS, self.joints.items()):
            canvas.create_line(legend_x, 23, legend_x + 13, 23, fill=color, width=3)
            canvas.create_text(legend_x + 17, 23, anchor="w", fill="#d1d5db",
                               font=("Segoe UI", 8), text=label)
            legend_x += 82

        if not active:
            canvas.create_text((left + right) / 2, (top + bottom) / 2,
                               fill="#fbbf24", font=("Segoe UI", 10),
                               text="Enable Physics mode to measure motor effort")
            return

        for color, name in zip(self.COLORS, self.joints):
            points = self.history.points(name, left, right, top, bottom, self.limit)
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2, smooth=False)
