# Majd
"""Plot received BLE samples using the existing preview drawing code."""

from __future__ import annotations

import time
import tkinter as tk

from ble_preview_draw import _draw_traces


class LiveSignalGraphs:
    """Share the simulator's reader and refresh charts at most 20 times a second."""

    def __init__(self, parent, calibrate_emg=None, calibrate_strength=None) -> None:
        self.window = tk.Toplevel(parent)
        self.window.title("LIMB live BLE graphs")
        self.window.geometry("1040x740+550+35")
        self.window.minsize(760, 580)
        self.window.protocol("WM_DELETE_WINDOW", self.window.withdraw)
        self.last_draw = 0.0
        self.status = tk.StringVar(master=self.window, value="Waiting for BLE samples")
        tk.Label(self.window, textvariable=self.status, anchor="w", wraplength=980).grid(
            row=0, column=0, columnspan=2, sticky="ew", padx=12, pady=8
        )
        self.zoom_emg = tk.BooleanVar(master=self.window, value=False)
        tk.Checkbutton(
            self.window,
            text="Auto-scale EMG (zoom small signals and noise)",
            variable=self.zoom_emg,
        ).grid(row=1, column=0, columnspan=2, sticky="w", padx=12)
        if calibrate_emg is not None or calibrate_strength is not None:
            actions = tk.Frame(self.window)
            actions.grid(row=1, column=1, sticky="e", padx=12)
        if calibrate_emg is not None:
            tk.Button(actions, text="1. Calibrate EMG at rest", command=calibrate_emg).pack(
                side="left"
            )
        if calibrate_strength is not None:
            tk.Button(
                actions, text="2. Calibrate strong grip (3 s)", command=calibrate_strength
            ).pack(side="left", padx=6)
        self.emg = self._chart("Raw EMG - ADC counts", 2, 0, colspan=2)
        self.imu = {}
        for column, role in enumerate(("shoulder", "wrist")):
            for row, signal in enumerate(("Acceleration (g)", "Gyroscope (deg/s)"), start=3):
                self.imu[role, row - 3] = self._chart(
                    f"{role.title()} - {signal} | X blue / Y orange / Z green", row, column
                )
        tk.Label(
            self.window,
            text="Live view only. Received samples are shown in order; gaps are counted above.",
            anchor="w",
            fg="#667085",
        ).grid(row=5, column=0, columnspan=2, sticky="ew", padx=12, pady=8)
        for column in range(2):
            self.window.columnconfigure(column, weight=1)
        for row in range(2, 5):
            self.window.rowconfigure(row, weight=1)

    def _chart(self, title: str, row: int, column: int, colspan: int = 1):
        frame = tk.Frame(self.window)
        frame.grid(row=row, column=column, columnspan=colspan, sticky="nsew", padx=12, pady=4)
        tk.Label(frame, text=title, anchor="w").pack(fill="x")
        canvas = tk.Canvas(frame, height=150, bg="white", highlightthickness=1)
        canvas.pack(fill="both", expand=True)
        return canvas

    def show(self) -> None:
        self.window.deiconify()
        self.window.lift()

    def update(self, reader) -> None:
        now = time.monotonic()
        if now - self.last_draw < 0.05 or not self.window.winfo_viewable():
            return
        self.last_draw = now
        data = reader.graph_data()
        health = data["health"]
        missing = sum(stream["missing_packets"] for stream in health.values())
        errors = sum(stream["decode_errors"] for stream in health.values())
        gaps = sum(stream["sequence_gaps"] for stream in health.values())
        activation = data["activation"]
        grip = "calibrating / waiting" if activation is None else f"{activation * 100:.1f}%"
        envelope = data.get("emg_envelope")
        signal_levels = (
            f" | RMS {envelope:.1f}, rest {data.get('emg_rest', 0):.1f} ADC"
            if envelope is not None
            else ""
        )
        self.status.set(
            f"BLE {data['status']} | grip activation {grip} | missing {missing} | "
            f"sequence gaps {gaps} | decode errors {errors}"
            + f" | {data.get('emg_calibration', '')}"
            + signal_levels
            + (f" | {data['error']}" if data["error"] else "")
        )
        _draw_traces(
            self.emg,
            [data["emg"]],
            unit="ADC",
            sample_period=data["emg_period"],
            limits=None if self.zoom_emg.get() else (0.0, 4095.0),
        )
        for (role, signal), canvas in self.imu.items():
            samples = data["imus"][role]
            traces = [
                [sample[axis] for sample in samples] for axis in range(signal * 3, signal * 3 + 3)
            ]
            _draw_traces(canvas, traces, unit="g" if signal == 0 else "deg/s")
