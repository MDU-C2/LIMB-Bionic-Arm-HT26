# Christian och Majd
"""Plot and inspect original/DMP angles with explicit time scaling and error units."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np

from aurora.json_utils import read_json_object
from aurora.paths import REPOSITORY_ROOT

os.environ.setdefault("MPLCONFIGDIR", str(REPOSITORY_ROOT / "outputs/cache/matplotlib"))


def comparison_figure(directory, *, duration_scale=None):
    from matplotlib.figure import Figure

    directory = Path(directory)
    report = read_json_object(directory / "experiment.json")
    with np.load(directory / "comparison.npz", allow_pickle=False) as data:
        time, original, generated = data["time_s"], data["q_demo_rad"], data["q_fit_rad"]
    scale = report["duration_scale"] if duration_scale is None else duration_scale
    if not np.isfinite(scale) or not 0.25 <= scale <= 4:
        raise ValueError("Duration multiplier must be between 0.25 and 4")
    original, generated = np.rad2deg(original), np.rad2deg(generated)
    figure = Figure(figsize=(12, 9), constrained_layout=True)
    axes = figure.subplots(4, 2)
    labels = [
        "Elbow flexion",
        "Shoulder flexion",
        "Shoulder abduction",
        "Shoulder internal rotation",
    ]
    if report["source_kind"] == "cup_trial":
        labels[3] = "Shoulder rotation proxy (estimated)"
    marker = "SYNTHETIC — " if report.get("synthetic") else ""
    figure.suptitle(
        f"{marker}DMP comparison | duration ×{scale:g}\n"
        "Reconstruction of source estimates; error aligned by movement phase",
        fontsize=13,
    )
    for i, label in enumerate(labels):
        left, right = axes[i]
        left.plot(time, original[:, i], label="Original", color="#326da8", linewidth=1.8)
        left.plot(time * scale, generated[:, i], label="DMP", color="#df6d25", linestyle="--")
        left.set(title=label, ylabel="Angle (degrees)")
        error = generated[:, i] - original[:, i]
        rmse = np.sqrt(np.mean(error**2))
        right.plot(time / time[-1], error, color="#a83d56")
        right.axhline(0, color="gray", linewidth=0.6)
        right.set(
            title=f"RMSE {rmse:.2f}° | maximum {np.max(np.abs(error)):.2f}°",
            ylabel="DMP − original (degrees)",
        )
        for axis in (left, right):
            axis.grid(alpha=0.2)
        left.legend(loc="best", fontsize=8)
    axes[-1, 0].set_xlabel("Time (seconds)")
    axes[-1, 1].set_xlabel("Movement phase (0–1)")
    return figure


def save_comparison(directory):
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    figure = comparison_figure(directory)
    FigureCanvasAgg(figure)
    figure.savefig(Path(directory) / "comparison.png", dpi=140)
    figure.clear()


def show_comparison(directory):
    import tkinter as tk
    from tkinter import ttk

    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

    directory = Path(directory).resolve()
    report = read_json_object(directory / "experiment.json")
    if report.get("status") != "complete":
        raise ValueError("Choose a completed DMP experiment folder")
    window = tk.Tk()
    window.title("AURORA | DMP comparison")
    window.geometry("1180x900")
    controls = ttk.Frame(window, padding=8)
    controls.pack(fill="x")
    ttk.Label(controls, text="Movement duration multiplier").pack(side="left")
    value = tk.StringVar(value=f"{report['duration_scale']:g}")
    choices = sorted({0.5, 1.0, 2.0, float(report["duration_scale"])})
    selector = ttk.Combobox(
        controls, textvariable=value, values=[f"{x:g}" for x in choices], state="readonly", width=7
    )
    selector.pack(side="left", padx=8)
    status = tk.StringVar()
    ttk.Label(controls, textvariable=status).pack(side="left")
    message = "SYNTHETIC EXERCISE. " if report.get("synthetic") else ""
    message += " ".join(report.get("warnings", []))
    ttk.Label(window, text=message, wraplength=1100, padding=8).pack(fill="x")
    ttk.Label(
        window,
        text="Simulator: Play DMP trajectory → choose this folder. "
        "Its duration multiplier is relative to the exported duration.\n" + str(directory),
        wraplength=1100,
        padding=8,
    ).pack(fill="x")
    plot = ttk.Frame(window)
    plot.pack(fill="both", expand=True)
    figure = comparison_figure(directory)
    canvas = FigureCanvasTkAgg(figure, master=plot)
    toolbar = NavigationToolbar2Tk(canvas, plot)
    canvas.get_tk_widget().pack(fill="both", expand=True)
    original_time = np.array(figure.axes[0].lines[0].get_xdata())

    def redraw(*_):
        scale = float(value.get())
        for axis in figure.axes[::2]:
            axis.lines[1].set_xdata(original_time * scale)
            axis.relim()
            axis.autoscale_view()
        marker = "SYNTHETIC — " if report.get("synthetic") else ""
        figure.suptitle(
            f"{marker}DMP comparison | duration ×{scale:g}\n"
            "Reconstruction of source estimates; error aligned by movement phase",
            fontsize=13,
        )
        toolbar.update()
        canvas.draw_idle()
        status.set(
            f"Original: {report['original_duration_s']:.2f}s  |  DMP: "
            f"{report['original_duration_s'] * scale:.2f}s (view only; saved experiment unchanged)"
        )

    selector.bind("<<ComboboxSelected>>", redraw)
    redraw()
    window.mainloop()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.headless:
            save_comparison(args.path)
        else:
            show_comparison(args.path)
        return 0
    except (OSError, ValueError, KeyError) as error:
        print(f"Cannot open DMP comparison: {error}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
