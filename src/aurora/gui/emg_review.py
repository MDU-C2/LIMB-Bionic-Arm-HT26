# Majd
"""Review raw EMG intervals with the saved trial video."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from aurora.json_utils import read_json_object
from aurora.recording.emg_features import PHASE_NAMES, load_emg
from aurora.recording.emg_labels import save_labels
from aurora.recording.saved_signals import Trace, emg_file, video_frame_times, with_gaps


class EmgReview:
    def __init__(self, source: Path):
        import tkinter as tk
        from tkinter import messagebox, ttk

        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        from matplotlib.figure import Figure

        self.tk, self.ttk, self.messagebox = tk, ttk, messagebox
        self.source = emg_file(source)
        self.times, self.raw = load_emg(self.source)
        if len(self.times) < 2 or not np.all(np.diff(self.times) > 0):
            raise ValueError("Label review needs increasing device timestamps.")
        self.seconds = (self.times - self.times[0]) / 1e6
        self.capture = None
        self.camera_start = None
        self.frame_times = None
        self.host = None
        with self.source.open(encoding="utf-8-sig", newline="") as stream:
            stamps = [row.get("synced_host_monotonic_ns", "") for row in csv.DictReader(stream)]
        if len(stamps) == len(self.raw) and all(stamps):
            host = np.asarray(stamps, dtype=np.int64)
            if np.all(host > 0) and np.all(np.diff(host) > 0):
                self.host = host
        trial_file = self.source.parent.parent / "trial.json"
        trial = read_json_object(trial_file) if trial_file.is_file() else {}
        self.window = tk.Tk()
        self.window.title("AURORA | Review EMG labels")
        self.window.geometry("1220x830")
        self.window.columnconfigure(0, weight=1)
        self.window.rowconfigure(2, weight=1)
        self.subject = tk.StringVar(value=trial.get("subject_id", ""))
        self.trial = tk.StringVar(value=trial.get("trial_id", self.source.parent.name))
        self.phase = tk.StringVar(value="rest")
        self.start, self.end = tk.StringVar(value="0"), tk.StringVar(value="1")
        self.status = tk.StringVar(
            value="Select an interval on the signal. Check the video, then add its phase."
        )
        self.segments = []
        controls = ttk.Frame(self.window, padding=8)
        controls.grid(row=0, column=0, sticky="ew")
        for column, (title, variable) in enumerate(
            (("Subject", self.subject), ("Trial", self.trial))
        ):
            ttk.Label(controls, text=title).grid(row=0, column=column * 2, padx=5)
            ttk.Entry(controls, textvariable=variable, width=18).grid(row=0, column=column * 2 + 1)
        ttk.Label(self.window, textvariable=self.status, padding=8, wraplength=1150).grid(
            row=1, column=0, sticky="w"
        )
        body = ttk.Panedwindow(self.window, orient="horizontal")
        body.grid(row=2, column=0, sticky="nsew")
        plot = ttk.Frame(body)
        video = ttk.Frame(body, padding=8)
        body.add(plot, weight=3)
        body.add(video, weight=2)
        self.figure = Figure(figsize=(7, 4))
        self.axis = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot)
        NavigationToolbar2Tk(self.canvas, plot)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.video_label = ttk.Label(video, text="No synchronized trial video.")
        self.video_label.pack(fill="both", expand=True)
        self.video_status = ttk.Label(video, text="", wraplength=400)
        self.video_status.pack()
        ttk.Scale(video, from_=0, to=self.seconds[-1], command=self._seek).pack(fill="x")
        self._open_video(trial_file.parent / "camera")
        edit = ttk.Frame(self.window, padding=8)
        edit.grid(row=3, column=0, sticky="ew")
        for column, (title, variable) in enumerate(
            (("Start (s)", self.start), ("End (s)", self.end))
        ):
            ttk.Label(edit, text=title).grid(row=0, column=column * 2, padx=5)
            ttk.Entry(edit, textvariable=variable, width=10).grid(row=0, column=column * 2 + 1)
        ttk.Combobox(
            edit, textvariable=self.phase, values=PHASE_NAMES, state="readonly", width=10
        ).grid(row=0, column=4, padx=8)
        ttk.Button(edit, text="Add interval", command=self._add).grid(row=0, column=5, padx=5)
        ttk.Button(edit, text="Remove selected", command=self._remove).grid(row=0, column=6, padx=5)
        self.tree = ttk.Treeview(
            self.window, columns=("start", "end", "phase"), show="headings", height=6
        )
        for key in ("start", "end", "phase"):
            self.tree.heading(key, text=key.capitalize())
        self.tree.grid(row=4, column=0, sticky="ew", padx=8)
        self.tree.bind("<<TreeviewSelect>>", self._selected)
        actions = ttk.Frame(self.window, padding=8)
        actions.grid(row=5, column=0, sticky="ew")
        for text, callback in (
            ("Load cue suggestions", self._load_cues),
            ("Save draft", lambda: self._save(False)),
            ("Save reviewed labels", lambda: self._save(True)),
        ):
            ttk.Button(actions, text=text, command=callback).pack(side="left", padx=5)
        labels = self.source.with_name(self.source.stem + ".labels.json")
        if labels.is_file():
            from aurora.recording.emg_labels import digest, validate_segments

            document = read_json_object(labels)
            if document.get("source_sha256") != digest(self.source):
                raise ValueError("Saved labels belong to a different raw recording.")
            self.segments = validate_segments(document["segments"], self.seconds[-1])
            self.subject.set(document["subject_id"])
            self.trial.set(document["trial_id"])
        self._redraw()
        self.window.protocol("WM_DELETE_WINDOW", self._close)

    def _open_video(self, directory: Path):
        if self.host is None or not (directory / "meta.json").is_file():
            return
        metadata = read_json_object(directory / "meta.json")
        self.camera_start = metadata.get("started_host_monotonic_ns")
        if not isinstance(self.camera_start, int) or not (directory / "video.mp4").is_file():
            return
        import cv2

        try:
            self.frame_times = video_frame_times(directory)
        except (OSError, ValueError) as error:
            self.video_status.configure(text=f"Video timestamp mapping unavailable: {error}")
            return
        self.cv2 = cv2
        self.capture = cv2.VideoCapture(str(directory / "video.mp4"))
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = None
            return
        self._seek(0)

    def _seek(self, seconds):
        if self.capture is None:
            return
        from PIL import Image, ImageTk

        index = min(len(self.host) - 1, int(np.searchsorted(self.seconds, float(seconds))))
        host_ns = int(self.host[index])
        video_s = (host_ns - self.camera_start) / 1e9
        if video_s < 0:
            self.video_label.configure(image="", text="No saved frame at this time.")
            self.video_status.configure(text="This interval is before the saved video.")
            return
        if self.frame_times is None:
            self.capture.set(self.cv2.CAP_PROP_POS_MSEC, video_s * 1000)
            alignment = "Legacy video time; receipt-to-frame mapping unavailable."
        else:
            stamps, frame_indices = self.frame_times
            # Pick an already received frame; a future frame could suggest early motion.
            position = int(np.searchsorted(stamps, host_ns, side="right")) - 1
            if position < 0:
                self.video_label.configure(image="", text="No saved frame at this time.")
                self.video_status.configure(text="This interval is before the first saved frame.")
                return
            self.capture.set(self.cv2.CAP_PROP_POS_FRAMES, int(frame_indices[position]))
            age_ms = (host_ns - int(stamps[position])) / 1e6
            alignment = f"Recorded frame received {age_ms:.1f} ms earlier."
        ok, frame = self.capture.read()
        if not ok:
            self.video_label.configure(image="", text="No saved frame at this time.")
            self.video_status.configure(text="No saved frame at this time.")
            return
        image = Image.fromarray(self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB))
        image.thumbnail((440, 330))
        self.image = ImageTk.PhotoImage(image, master=self.window)
        self.video_label.configure(image=self.image, text="")
        self.video_status.configure(
            text=f"EMG {float(seconds):.3f} s. {alignment} "
            "Camera times mark host receipt, not exposure."
        )

    def _select_span(self, start, end):
        self.start.set(f"{max(0, start):.6f}")
        self.end.set(f"{min(self.seconds[-1], end):.6f}")
        self._seek(start)

    def _add(self):
        from aurora.recording.emg_labels import validate_segments

        try:
            proposed = self.segments + [
                {
                    "start_s": float(self.start.get()),
                    "end_s": float(self.end.get()),
                    "phase": self.phase.get(),
                }
            ]
            self.segments = validate_segments(
                sorted(proposed, key=lambda s: s["start_s"]), self.seconds[-1]
            )
        except ValueError as error:
            self.messagebox.showerror("Check interval", str(error), parent=self.window)
            return
        self.status.set("Interval added. Unlabelled sections will be excluded from scoring.")
        self._redraw()

    def _remove(self):
        selection = self.tree.selection()
        if selection:
            self.segments.pop(int(selection[0]))
            self._redraw()

    def _selected(self, _event):
        selection = self.tree.selection()
        if selection:
            segment = self.segments[int(selection[0])]
            self.start.set(str(segment["start_s"]))
            self.end.set(str(segment["end_s"]))
            self.phase.set(segment["phase"])
            self._seek(segment["start_s"])

    def _load_cues(self):
        path = self.source.parent / "emg_trial.json"
        if self.host is None or not path.is_file():
            self.status.set(
                "No synchronized grip cues. Mark intervals by checking the signal and video."
            )
            return
        if self.segments:
            self.status.set("Remove existing intervals before importing cue suggestions.")
            return
        from aurora.recording.emg_labels import validate_segments

        try:
            document = read_json_object(path)
            origin = document["start_host_monotonic_ns"]
            if (
                type(origin) is not int
                or origin <= 0
                or not isinstance(document.get("segments"), list)
            ):
                raise ValueError("Grip cues need a positive host start time and phase intervals.")
            proposed = []
            for segment in document["segments"]:
                start_ns = origin + round(float(segment["start_s"]) * 1e9)
                end_ns = origin + round(float(segment["end_s"]) * 1e9)
                start, end = np.interp([start_ns, end_ns], self.host, self.seconds)
                if start < end:
                    proposed.append(
                        {"start_s": float(start), "end_s": float(end), "phase": segment["phase"]}
                    )
            self.segments = validate_segments(proposed, self.seconds[-1])
        except (OSError, KeyError, TypeError, ValueError, OverflowError) as error:
            self.messagebox.showerror(
                "Could not load cue suggestions", str(error), parent=self.window
            )
            return
        self.status.set(
            "Cue suggestions loaded. Check every boundary; these are not reviewed labels yet."
        )
        self._redraw()

    def _redraw(self):
        from matplotlib.widgets import SpanSelector

        if hasattr(self, "selector"):
            self.selector.disconnect_events()
        self.tree.delete(*self.tree.get_children())
        limits = self.axis.get_xlim() if self.axis.has_data() else None
        self.axis.clear()
        self.axis.plot(
            *with_gaps(Trace("Raw EMG", self.seconds, self.raw)), linewidth=0.5, color="0.25"
        )
        colors = dict(
            zip(PHASE_NAMES, ("tab:blue", "tab:green", "tab:orange", "tab:red"), strict=True)
        )
        for index, segment in enumerate(self.segments):
            self.tree.insert(
                "",
                "end",
                iid=str(index),
                values=(f"{segment['start_s']:.3f}", f"{segment['end_s']:.3f}", segment["phase"]),
            )
            self.axis.axvspan(
                segment["start_s"], segment["end_s"], color=colors[segment["phase"]], alpha=0.2
            )
        self.axis.set(
            xlabel="Seconds from first EMG sample", ylabel="Raw ADC counts", title=self.source.name
        )
        if limits:
            self.axis.set_xlim(limits)
        self.figure.tight_layout()
        self.selector = SpanSelector(
            self.axis,
            self._select_span,
            "horizontal",
            useblit=True,
            props={"alpha": 0.25, "facecolor": "tab:blue"},
            interactive=True,
        )
        self.canvas.draw_idle()

    def _save(self, reviewed):
        try:
            path = save_labels(
                self.source, self.subject.get(), self.trial.get(), self.segments, reviewed=reviewed
            )
            self.status.set(
                f"Saved {'reviewed labels' if reviewed else 'draft'}: {path.name}. Raw data unchanged."
            )
        except (OSError, ValueError) as error:
            self.messagebox.showerror("Could not save labels", str(error), parent=self.window)

    def _close(self):
        if self.capture is not None:
            self.capture.release()
        self.window.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True, help="trial.json or raw EMG CSV")
    args = parser.parse_args()
    try:
        review = EmgReview(args.file)
        review.window.mainloop()
    except (OSError, ValueError, KeyError, ImportError) as error:
        parser.exit(1, f"EMG review: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
