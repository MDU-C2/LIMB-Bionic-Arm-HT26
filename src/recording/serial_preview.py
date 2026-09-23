"""Small live window for a generic newline-delimited serial stream."""

from __future__ import annotations

from collections import deque
import json
import math
import tkinter as tk


COLORS = ("#2563eb", "#d97706", "#159669", "#7c3aed", "#db2777")


def numeric_values(text: str, limit: int = 8) -> dict[str, float]:
    """Return numeric values that can be graphed from one serial line."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        try:
            value = float(text)
        except ValueError:
            return {}
        return {"value": value} if math.isfinite(value) else {}

    values: dict[str, float] = {}

    def visit(value, path: str) -> None:
        if len(values) >= limit:
            return
        if isinstance(value, bool):
            return
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values[path or "value"] = float(value)
        elif isinstance(value, dict):
            for key, child in value.items():
                visit(child, f"{path}.{key}" if path else str(key))
        elif isinstance(value, list):
            for index, child in enumerate(value):
                visit(child, f"{path}[{index}]" if path else f"[{index}]")

    visit(data, "")
    return values


class SerialPreview:
    """Display recent text and graph numeric values without saving data."""

    def __init__(self, port: str, baud: int) -> None:
        self.root = tk.Tk()
        self.root.title("AURORA serial sensor stream - NOT RECORDING")
        self.root.geometry("920x650")
        self.root.minsize(620, 440)
        self.closed = False
        self.messages = 0
        self.histories: dict[str, deque[float]] = {}
        self.status = tk.StringVar(value=f"{port} at {baud} baud | waiting for data")
        self.legend = tk.StringVar(value="No numeric values detected yet")

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda _event: self.close())
        frame = tk.Frame(self.root, padx=14, pady=12)
        frame.pack(fill="both", expand=True)
        tk.Label(
            frame, textvariable=self.status, font=("Segoe UI", 11, "bold"), anchor="w"
        ).pack(fill="x")
        tk.Label(
            frame,
            text="Live preview only. No files are saved.",
            anchor="w",
            fg="#667085",
        ).pack(fill="x", pady=(0, 10))
        tk.Label(frame, text="Detected numeric values", anchor="w").pack(fill="x")
        self.canvas = tk.Canvas(
            frame,
            height=230,
            bg="white",
            highlightbackground="#ccd3df",
            highlightthickness=1,
        )
        self.canvas.pack(fill="both", expand=True, pady=(4, 4))
        tk.Label(
            frame, textvariable=self.legend, anchor="w", font=("Consolas", 9)
        ).pack(fill="x", pady=(0, 10))
        tk.Label(frame, text="Recent serial lines", anchor="w").pack(fill="x")
        self.log = tk.Text(frame, height=10, wrap="none", font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, pady=(4, 0))
        self.log.configure(state="disabled")

    def add_line(self, timestamp: str, text: str) -> None:
        """Add one line and update any numeric traces found in it."""
        self.messages += 1
        self.status.set(f"Receiving data | {self.messages} messages")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{timestamp}] {text}\n")
        if int(self.log.index("end-1c").split(".")[0]) > 201:
            self.log.delete("1.0", "2.0")
        self.log.see("end")
        self.log.configure(state="disabled")

        for name, value in numeric_values(text).items():
            self.histories.setdefault(name, deque(maxlen=200)).append(value)
        self._draw()

    def _draw(self) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 2)
        height = max(self.canvas.winfo_height(), 2)
        self.canvas.create_line(0, height / 2, width, height / 2, fill="#d6dce5")
        shown = list(self.histories.items())[: len(COLORS)]
        all_values = [value for _name, history in shown for value in history]
        if not all_values:
            self.canvas.create_text(
                12,
                height / 2,
                text="Waiting for numeric JSON values or number-only lines...",
                anchor="w",
                fill="#667085",
            )
            return
        low, high = min(all_values), max(all_values)
        if high - low < 1e-9:
            low -= 1.0
            high += 1.0
        for index, (_name, history) in enumerate(shown):
            if len(history) < 2:
                continue
            points: list[float] = []
            for sample, value in enumerate(history):
                points.extend(
                    (
                        10 + sample * (width - 20) / (len(history) - 1),
                        8 + (high - value) * (height - 16) / (high - low),
                    )
                )
            self.canvas.create_line(*points, fill=COLORS[index], width=2)
        self.legend.set(
            "  |  ".join(
                f"{name}: {history[-1]:.3g}"
                for name, history in shown
                if history
            )
        )

    def refresh(self) -> bool:
        """Process window events and report whether the window is still open."""
        if self.closed:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self.closed = True
        return not self.closed

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass
