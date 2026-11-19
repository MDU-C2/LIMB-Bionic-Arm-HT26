"""Hardware-free signal learning and recording-setup rehearsal."""

from __future__ import annotations

import argparse
import contextlib
import math
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from aurora.gui.guidance import GuidancePane
from aurora.gui.project_support import open_path
from aurora.gui.sensor_plot import AXIS_COLORS, draw_traces
from aurora.paths import REPOSITORY_ROOT
from aurora.recording.common import enable_ctrl_break_as_interrupt
from aurora.recording.cup_protocol import CupReferenceGuide
from aurora.sensors.sensor_lesson import FAULTS, LESSONS, MOUNTINGS, STEP_S, SensorLesson

EXPLANATIONS = {
    "STILL": "Still does not mean zero acceleration: gravity remains about 1 g in total. Gyros should settle near zero.",
    "RAISE": "Both segments rotate while the elbow bend stays fixed. Raw XYZ channels depend on each mounting.",
    "BEND": "The forearm rotates while the upper arm stays still. After stopping, gyro returns to zero at the new elbow angle.",
    "TURN": "Forearm angular velocity changes with palm turning. Upper-arm gyro should remain quiet in this ideal movement.",
    "RETURN": "Return to the original pose and compare the signals. A remaining difference may indicate bias, noise or a moved sensor.",
    "RELAX": "Relaxation gives a small, irregular noise floor. It is a baseline, not a perfectly flat zero line.",
    "GRIP": "The raw EMG fluctuations grow during contraction even when the arm is still. RMS smooths their amplitude; it is not measured grip force.",
    "RELEASE": "The fluctuations and RMS should fall towards the resting level. Filtering takes time to settle.",
    "MANUAL": "Move one joint at a time. Known human pose, estimated angles and bounded robot commands have different meanings.",
    "CUP": "Illustrative hand 1 → pickup 2 → place/release 3 → empty hand 4. Requested cues do not verify a real participant's actions.",
}
MOVEMENT_NAMES = dict(
    STILL="Still reference",
    RAISE="Upper-arm raising",
    BEND="Elbow bending",
    TURN="Forearm turning",
    RETURN="Return to reference",
    RELAX="Relaxed EMG",
    GRIP="Firm grip",
    RELEASE="Release",
    MANUAL="Manual movement",
    CUP="Cup transfer",
)


class LessonWorker:
    """One processing owner; Tk only reads bounded published history."""

    def __init__(self, side, output):
        self.side, self.output = side, output
        self.commands = queue.Queue(maxsize=8)
        self.configuration = queue.Queue(maxsize=1)
        self.frames = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    @staticmethod
    def latest(mailbox, value):
        with contextlib.suppress(queue.Empty):
            mailbox.get_nowait()
        mailbox.put_nowait(value)

    def run(self):
        lesson = None
        try:
            lesson = SensorLesson(self.side)
            paused, speed, debt = False, 1.0, 0.0
            notice, output = "", None
            previous = published = time.monotonic()
            while not self.stop.is_set():
                now = time.monotonic()
                elapsed, previous = now - previous, now
                try:
                    values = self.configuration.get_nowait()
                    speed = float(values.pop("speed", speed))
                    lesson.configure(values)
                    if paused:
                        notice = "Settings apply on the next sample. Advance or Resume; inspected history stays unchanged."
                except queue.Empty:
                    pass
                for _ in range(8):
                    try:
                        command = self.commands.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        notice = ""
                        if command == "pause":
                            paused = not paused
                            debt = 0.0
                        elif command == "step":
                            paused, debt = True, 0.0
                            lesson.advance()
                        elif command == "reset":
                            settings = dict(lesson.settings)
                            lesson.close()
                            lesson = SensorLesson(self.side)
                            lesson.configure(settings)
                            paused, debt = False, 0.0
                        elif command == "prepare":
                            lesson.start_preparation()
                            paused = False
                        elif command == "explore":
                            lesson.guide = None
                            lesson.origin = lesson.t
                            paused = False
                        elif command == "next":
                            lesson.start_step()
                            paused = False
                        elif command == "trial":
                            lesson.start_trial()
                            paused = False
                        elif command == "export":
                            paused, debt = True, 0.0
                            output = lesson.export(self.output)
                            notice = f"Saved synthetic signal window: {output}"
                            print(notice, flush=True)
                    except (OSError, ValueError) as error:
                        notice = str(error)
                if not paused:
                    debt += elapsed * speed
                    # Catch up in bounded batches without dropping virtual samples.
                    for _ in range(min(8, int(debt / STEP_S))):
                        lesson.advance()
                        debt -= STEP_S
                if now - published >= 0.05:
                    self.latest(
                        self.frames,
                        dict(
                            rows=tuple(lesson.history),
                            paused=paused,
                            notice=notice,
                            output=output,
                            backlog=debt,
                            next_ready=isinstance(lesson.guide, CupReferenceGuide)
                            and lesson.guide.phase == "reference_prepare",
                        ),
                    )
                    published = now
                self.stop.wait(0.005)
        except Exception as error:
            self.latest(self.frames, {"error": f"Learning source stopped: {error}"})
        finally:
            if lesson is not None:
                lesson.close()


class SensorLearning(ttk.Frame):
    def __init__(self, window, *, side="left", output=None, stop_requested=None):
        super().__init__(window, padding=10)
        self.window, self.side = window, side
        self.pack(fill="both", expand=True)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)
        self.rows, self.paused, self.last_output = (), False, None
        self.guide_window = self.guide_pane = None
        self.closing = False
        self.stop_requested = stop_requested
        self.next_ready = False
        self.variables = {}
        self.worker = LessonWorker(
            side, output or REPOSITORY_ROOT / "outputs/diagnostics/sensor_learning"
        )
        ttk.Label(
            self,
            text=f"SIMULATED SIGNALS · {side} arm · No hardware connection",
            font=("Segoe UI", 16, "bold"),
            foreground="#185a78",
        ).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(
            self,
            text="Learn expected signals and rehearse setup. This cannot establish real mounting, electrode quality or tracking accuracy.",
            wraplength=1150,
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(2, 8))
        sidebar = ttk.Frame(self, padding=(0, 0, 12, 0))
        sidebar.grid(row=2, column=0, sticky="nsw")
        tabs = ttk.Notebook(sidebar)
        tabs.grid(row=0, column=0, sticky="nsew")
        controls, setup = ttk.Frame(tabs, padding=6), ttk.Frame(tabs, padding=6)
        tabs.add(controls, text="Explore signals")
        tabs.add(setup, text="Setup / Save")
        actions = ttk.Frame(sidebar)
        actions.grid(row=1, column=0, sticky="ew", pady=5)
        self._choice(
            controls,
            "lesson",
            "Movement",
            tuple(MOVEMENT_NAMES.values()),
            MOVEMENT_NAMES["BEND"],
            0,
        )
        for row, key, title in (
            (1, "upper_mount", "Upper-arm sensor mounting"),
            (2, "forearm_mount", "Forearm sensor mounting"),
            (3, "fault", "Signal condition"),
        ):
            self._choice(
                controls,
                key,
                title,
                FAULTS if key == "fault" else MOUNTINGS,
                "None" if key == "fault" else MOUNTINGS[0],
                row,
            )
        for row, key, title, low, high, value in (
            (4, "shoulder", "Manual shoulder flexion (°)", 0, 75, 0),
            (5, "elbow", "Manual elbow flexion (°)", 0, 140, 90),
            (6, "turn", "Manual forearm turning (°)", -90, 90, 0),
            (7, "effort", "Simulated contraction (0–1)", 0, 1, 0),
        ):
            variable = self.variables[key] = tk.DoubleVar(self, value=value)
            ttk.Label(controls, text=title).grid(row=row * 2, column=0, sticky="w", pady=(7, 0))
            ttk.Scale(
                controls,
                from_=low,
                to=high,
                variable=variable,
                command=lambda value: self.configure_source(),
            ).grid(row=row * 2 + 1, column=0, sticky="ew")
        self.variables["speed"] = tk.DoubleVar(self, value=1)
        speeds = ttk.Frame(controls)
        speeds.grid(row=16, column=0, sticky="w", pady=6)
        for label, value in (("¼ speed", 0.25), ("1×", 1), ("2×", 2)):
            ttk.Radiobutton(
                speeds,
                text=label,
                value=value,
                variable=self.variables["speed"],
                command=self.configure_source,
            ).pack(side="left")
        self.variables["noise"] = tk.BooleanVar(self, value=False)
        ttk.Checkbutton(
            controls,
            text="Add sensor noise",
            variable=self.variables["noise"],
            command=self.configure_source,
        ).grid(row=17, column=0, sticky="w")
        self.pause_button = ttk.Button(actions, text="Pause", command=lambda: self.command("pause"))
        self.pause_button.grid(row=0, column=0, sticky="ew", padx=3)
        for row, label, command in (
            (0, "Rehearse mounting + EMG", "prepare"),
            (2, "Rehearse cup countdown", "trial"),
            (4, "Save last ≤20 s of signals", "export"),
        ):
            ttk.Button(setup, text=label, command=lambda key=command: self.command(key)).grid(
                row=row, column=0, sticky="ew", pady=3
            )
        ttk.Button(actions, text="Advance 10 ms", command=lambda: self.command("step")).grid(
            row=0, column=1, padx=3
        )
        ttk.Button(actions, text="Restart experiment", command=lambda: self.command("reset")).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=4
        )
        self.next_button = ttk.Button(
            setup, text="Start preparation step", command=lambda: self.command("next")
        )
        self.next_button.grid(row=1, column=0, sticky="ew", pady=3)
        ttk.Button(setup, text="Open movement guide", command=self.open_guide).grid(
            row=3, column=0, sticky="ew", pady=3
        )
        self.output_button = ttk.Button(
            setup, text="Open export folder", command=self.open_output, state="disabled"
        )
        self.output_button.grid(row=5, column=0, sticky="ew", pady=3)
        ttk.Label(
            setup,
            text="Attach each IMU to its own segment, away from the joint. Match your board's actual axis markings.\n\nEMG enclosure and skin electrodes are separate placements. Red dots are illustrative; use your electrode manufacturer's instructions and confirm a rest/grip difference.\n\nRehearsal affects only the simulated source. Physical recording still needs real calibration.",
            wraplength=235,
        ).grid(row=6, column=0, sticky="w", pady=12)
        self.activation = ttk.Progressbar(sidebar, maximum=100)
        self.activation.grid(row=2, column=0, sticky="ew", pady=(5, 0))
        self.activation_label = ttk.Label(sidebar, text="EMG activation: waiting", wraplength=240)
        self.activation_label.grid(row=3, column=0, sticky="w")
        content = ttk.Frame(self)
        content.grid(row=2, column=1, sticky="nsew")
        content.columnconfigure((0, 1), weight=1)
        self.explanation = tk.StringVar(self)
        self.explanation_label = ttk.Label(content, textvariable=self.explanation, wraplength=900)
        self.explanation_label.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.arm = tk.Canvas(content, height=180, bg="#f4f7fb", highlightthickness=0)
        self.arm.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=5)
        self.readout = tk.StringVar(self)
        self.readout_label = ttk.Label(
            content, textvariable=self.readout, font=("Consolas", 10), wraplength=900
        )
        self.readout_label.grid(row=2, column=0, columnspan=2, sticky="ew")
        content.bind("<Configure>", self.resize_content)
        self.charts = {}
        for row, (key, title) in enumerate(
            (
                (("shoulder", 0), "Upper arm acceleration · XYZ [g]"),
                (("wrist", 0), "Forearm acceleration · XYZ [g]"),
                (("shoulder", 3), "Upper arm angular velocity · XYZ [°/s]"),
                (("wrist", 3), "Forearm angular velocity · XYZ [°/s]"),
                ("emg", "Raw EMG · last 200 ms · ADC counts"),
                ("rms", "EMG RMS · ADC counts (not grip force)"),
            )
        ):
            box = ttk.Frame(content)
            box.grid(row=3 + row // 2, column=row % 2, sticky="nsew", padx=4, pady=4)
            ttk.Label(box, text=title).pack(anchor="w")
            canvas = tk.Canvas(box, height=80, bg="white", highlightthickness=0)
            canvas.pack(fill="both", expand=True)
            self.charts[key] = canvas
            content.rowconfigure(3 + row // 2, weight=1)
        self.age = tk.DoubleVar(self, value=0)
        ttk.Label(
            self, text="Inspect history: seconds behind the latest simulated sample (pause first)"
        ).grid(row=3, column=0, columnspan=2, sticky="w")
        self.cursor = ttk.Scale(
            self, from_=0, to=20, variable=self.age, command=lambda value: self.render()
        )
        self.cursor.grid(row=4, column=0, columnspan=2, sticky="ew")
        self.status = tk.StringVar(self, value="Starting the offline source…")
        ttk.Label(self, textvariable=self.status, wraplength=1150).grid(
            row=5, column=0, columnspan=2, sticky="w", pady=5
        )
        window.protocol("WM_DELETE_WINDOW", self.close)
        self.timer = self.after(50, self.poll)
        self.configure_source()

    def _choice(self, parent, key, label, choices, default, row):
        ttk.Label(parent, text=label).grid(row=row * 2, column=0, sticky="w", pady=(7, 0))
        variable = self.variables[key] = tk.StringVar(self, value=default)
        box = ttk.Combobox(
            parent, textvariable=variable, values=choices, state="readonly", width=28
        )
        box.grid(row=row * 2 + 1, column=0, sticky="ew")

        def selected(event):
            self.configure_source()
            if key == "lesson":
                self.command("explore")

        box.bind("<<ComboboxSelected>>", selected)

    def resize_content(self, event):
        for label in (self.explanation_label, self.readout_label):
            label.configure(wraplength=max(300, event.width - 12))

    def configure_source(self):
        values = {key: value.get() for key, value in self.variables.items()}
        values["lesson"] = next(
            key for key, name in MOVEMENT_NAMES.items() if name == values["lesson"]
        )
        LessonWorker.latest(self.worker.configuration, values)

    def command(self, name):
        try:
            self.worker.commands.put_nowait(name)
            if name != "export":
                self.age.set(0)
        except queue.Full:
            self.status.set("Waiting for processing. Retry the action shortly.")

    def poll(self):
        if self.stop_requested is not None and self.stop_requested.is_set():
            self.close()
            return
        try:
            frame = self.worker.frames.get_nowait()
        except queue.Empty:
            frame = None
        if frame is not None:
            if "error" in frame:
                self.status.set(frame["error"])
            else:
                self.rows, self.paused, self.last_output = (
                    frame["rows"],
                    frame["paused"],
                    frame["output"],
                )
                self.next_ready = frame["next_ready"]
                self.pause_button.configure(text="Resume" if self.paused else "Pause")
                self.cursor.configure(state="normal" if self.paused else "disabled")
                if not self.paused:
                    self.age.set(0)
                self.output_button.configure(state="normal" if self.last_output else "disabled")
                self.render()
                if frame["notice"]:
                    self.status.set(frame["notice"])
                elif frame["backlog"] > 0.1:
                    self.status.set(
                        "Processing is behind wall time; simulated samples remain ordered and are not skipped."
                    )
        if not self.closing:
            self.timer = self.after(50, self.poll)

    def render(self):
        if not self.rows:
            return
        stamp = self.rows[-1]["time_s"] - self.age.get()
        index = max(0, min(len(self.rows) - 1, round((stamp - self.rows[0]["time_s"]) / STEP_S)))
        row = self.rows[index]
        recent = self.rows[max(0, index - 499) : index + 1]
        guide = row["guide"]
        self.next_button.configure(
            state="normal" if self.next_ready and self.age.get() < 0.005 else "disabled"
        )
        render_key = (id(self.rows[-1]), self.age.get(), self.winfo_width(), self.winfo_height())
        if render_key == getattr(self, "render_key", None):
            return
        self.render_key = render_key
        instruction = guide.get("instruction", LESSONS[row["movement"]])
        self.explanation.set(
            f"{guide.get('cue', MOVEMENT_NAMES[row['movement']])}\n{instruction}\n{EXPLANATIONS[row['movement']]}"
        )
        self.draw_arm(row)
        parts = []
        for name, label, robot in (
            ("shoulder_flexion", "Shoulder", "shoulder_y"),
            ("elbow_flexion", "Elbow", "elbow_x"),
            ("wrist_rotation", "Forearm turn", "wrist_rotation"),
        ):
            estimate = row["angles"].get(name)
            relative = row["relative"].get(name)
            measured = (
                f"estimate {estimate:.1f}°"
                if estimate is not None
                else f"relative Δ {relative:.1f}°"
                if relative is not None
                else "estimate unavailable"
            )
            command = row["targets"].get(robot)
            parts.append(
                f"{label}: known {row['true_angles'][name]:.1f}° | {measured} | robot {command:.1f}°"
                if command is not None
                else f"{label}: known {row['true_angles'][name]:.1f}° | {measured}"
            )
        self.readout.set("\n".join(parts))
        for key, canvas in self.charts.items():
            if isinstance(key, tuple):
                role, start = key
                traces = (
                    [
                        [
                            frame["raw"][role][axis]
                            if role in frame["received_roles"]
                            else math.nan
                            for frame in recent
                        ]
                        for axis in range(start, start + 3)
                    ]
                    if role in row["received_roles"]
                    else []
                )
                draw_traces(canvas, traces, unit="g" if start == 0 else "°/s", sample_period=STEP_S)
            elif key == "emg":
                values = [value for frame in recent[-20:] for value in frame["emg"]]
                draw_traces(canvas, [values], unit="ADC", limits=(0, 4095), sample_period=1 / 4000)
            else:
                draw_traces(
                    canvas,
                    [[frame["emg_rms"] for frame in recent]],
                    unit="ADC",
                    sample_period=STEP_S,
                )
        activation = row["activation"]
        self.activation["value"] = 100 * activation if activation is not None else 0
        self.activation_label.configure(
            text=f"EMG activation: {100 * activation:.0f}%\n{row['emg_state']['message']}"
            if activation is not None
            else f"EMG activation unavailable\n{row['emg_state']['message']}"
        )
        self.status.set(
            f"Simulation {row['time_s']:.2f} s · {row['fault']} · {row['fit']}\n{row['status']}"
        )
        if self.guide_pane is not None and self.guide_window.winfo_exists():
            self.guide_pane.show(
                guide
                or {"movement": row["movement"], "instruction": instruction, "cue": "SIMULATED"},
                self.side,
                "Rehearsal only. No physical calibration or participant recording.",
            )

    def draw_arm(self, row):
        c = self.arm
        c.delete("all")
        width = max(c.winfo_width(), 500)

        def project(point):
            x, y, z = point
            return width * 0.45 - x * 200 + (z + 1.6) * 220, 45 - y * 260

        points = row["points"]
        shoulder = points[f"{self.side}_shoulder"]
        elbow, wrist = points[f"{self.side}_elbow"], points[f"{self.side}_wrist"]
        c.create_oval(width * 0.5 - 18, 3, width * 0.5 + 18, 35, fill="#d7e3ee", outline="")
        c.create_line(
            *project(points["left_shoulder"]),
            *project(points["right_shoulder"]),
            fill="#a8b7c8",
            width=10,
        )
        for role, a, b, color, label in (
            ("shoulder", shoulder, elbow, "#7c3aed", "Upper-arm IMU · 0x6B"),
            ("wrist", elbow, wrist, "#db2777", "Forearm IMU · 0x6A"),
        ):
            c.create_line(*project(a), *project(b), fill="#71849a", width=13, capstyle="round")
            middle = [(first + second) / 2 for first, second in zip(a, b, strict=True)]
            x, y = project(middle)
            c.create_rectangle(x - 9, y - 9, x + 9, y + 9, fill=color, outline="white", width=2)
            axes = row["boards"][role]
            centre_x = width * (0.16 if role == "shoulder" else 0.82)
            centre_y = 110
            c.create_text(centre_x, 65, text=label, fill=color)
            c.create_rectangle(
                centre_x - 9,
                centre_y - 9,
                centre_x + 9,
                centre_y + 9,
                fill=color,
                outline="white",
                width=2,
            )
            for axis, axis_color in enumerate(AXIS_COLORS):
                end_x = centre_x + 42 * (-axes[0][axis] + 0.65 * axes[2][axis])
                end_y = centre_y - 42 * axes[1][axis]
                c.create_line(
                    centre_x, centre_y, end_x, end_y, fill=axis_color, width=2, arrow="last"
                )
                c.create_text(end_x + 7, end_y, text="XYZ"[axis], fill=axis_color)
        for fraction in (0.22, 0.35):
            point = [a + fraction * (b - a) for a, b in zip(elbow, wrist, strict=True)]
            x, y = project(point)
            c.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#b91c1c", outline="white")
        c.create_text(10, 18, text="Sensor X blue · Y orange · Z green", anchor="w", fill="#43566a")
        c.create_text(
            10,
            35,
            text="Mounts and electrodes are schematic. See Setup / Save.",
            anchor="w",
            fill="#43566a",
        )

    def open_guide(self):
        if self.guide_window is None or not self.guide_window.winfo_exists():
            self.guide_window = tk.Toplevel(self.window)
            self.guide_window.title("AURORA | Simulated preparation guide")
            self.guide_window.geometry("650x570")
            self.guide_pane = GuidancePane(self.guide_window, large=True)
            self.guide_pane.pack(fill="both", expand=True)
        self.guide_window.lift()
        self.render_key = None
        self.render()

    def open_output(self):
        if self.last_output:
            try:
                open_path(self.last_output)
            except OSError as error:
                messagebox.showerror("Cannot open export folder", str(error), parent=self.window)

    def close(self):
        if not self.closing:
            self.closing = True
            self.after_cancel(self.timer)
            self.worker.stop.set()
            self.close_deadline = time.monotonic() + 3
        if self.worker.thread.is_alive():
            if time.monotonic() > self.close_deadline:
                self.status.set(
                    "Source shutdown timed out. Retry Close after the pending action finishes."
                )
                return
            self.status.set("Closing the simulated source…")
            self.after(25, self.close)
        else:
            for callback in self.window.tk.call("after", "info"):
                self.window.tk.call("after", "cancel", callback)
            self.window.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--output-root", type=Path, default=REPOSITORY_ROOT / "outputs/diagnostics/sensor_learning"
    )
    args = parser.parse_args()
    stop_requested = threading.Event()
    enable_ctrl_break_as_interrupt(stdin_stop=True, on_stop=stop_requested.set)
    window = tk.Tk()
    window.title("AURORA | Learn signals and mounting")
    window.geometry(f"1250x{min(940, window.winfo_screenheight() - 100)}")
    window.minsize(1050, 720)
    app = SensorLearning(
        window, side=args.side, output=args.output_root, stop_requested=stop_requested
    )
    print("Synthetic signal learning ready; no hardware connections.", flush=True)
    try:
        window.mainloop()
    except KeyboardInterrupt:
        app.close()
        while app.worker.thread.is_alive():
            window.update()
            time.sleep(0.01)
    finally:
        app.worker.stop.set()


if __name__ == "__main__":
    main()
