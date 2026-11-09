"""Show the recording-quality report for one subject session."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from aurora.recording.subject_summary import loss_percent


def count_text(value):
    return "—" if value is None else f"{value:,}"


def packet_text(totals, sensor):
    lost, received = totals[f"{sensor}_lost"], totals[f"{sensor}_received"]
    percent = loss_percent(received, lost)
    return count_text(lost) + (f" ({percent:.2f}%)" if percent is not None else "")


def capture_details(row):
    return "\n".join(
        [
            f"{'Reference' if row['reference'] else 'Trial'} {row['id']} — {row['path']}",
            f"Received packets: IMU {count_text(row['imu_received'])}; EMG {count_text(row['emg_received'])}",
            f"Missing packets: IMU {packet_text(row, 'imu')}; EMG {packet_text(row, 'emg')}",
            f"EMG samples at ADC limits: {count_text(row['emg_rail_samples'])} / {count_text(row['emg_samples'])}",
            f"Video sequence gaps: {count_text(row['video_gaps'])}; host queue losses: {count_text(row['video_queue_losses'])}",
            f"Depth sequence gaps: {count_text(row['depth_gaps'])}; host queue losses: {count_text(row['depth_queue_losses'])}",
            "",
            *(row["issues"] or ["No quality warnings reported for this capture."]),
        ]
    )


class SubjectSummaryWindow:
    def __init__(self, parent, report, path=None):
        self.report = report
        self.window = tk.Toplevel(parent)
        self.window.title(f"AURORA | {report['subject_id']} recording quality")
        self.window.geometry("1150x740")
        self.window.minsize(850, 550)
        body = ttk.Frame(self.window, padding=16)
        body.pack(fill="both", expand=True)
        ttk.Label(
            body,
            text=f"Subject {report['subject_id']} — recording quality",
            font=("Segoe UI", 20, "bold"),
        ).pack(anchor="w")
        count = report["trial_count"]
        ttk.Label(
            body,
            text=(
                f"This session: {count} trial(s) — {report['complete_trials']} capture complete, "
                f"{count - report['complete_trials']} incomplete, {report['trials_with_warnings']} with warnings."
            ),
            wraplength=1050,
        ).pack(anchor="w", pady=(8, 4))
        totals = report["trial_totals"]
        ttk.Label(
            body,
            text=f"Trial packet loss: IMU {packet_text(totals, 'imu')} | EMG {packet_text(totals, 'emg')} | EMG samples at ADC limits: {count_text(totals['emg_rail_samples'])}",
            wraplength=1050,
        ).pack(anchor="w", pady=4)
        totals = report["reference_totals"]
        ttk.Label(
            body,
            text=f"References: {report['reference_count']} | Reference packet loss: IMU {packet_text(totals, 'imu')} | EMG {packet_text(totals, 'emg')}",
            wraplength=1050,
        ).pack(anchor="w", pady=4)
        errors = report["session_errors"]
        self.session_status = ttk.Label(
            body,
            text="Session errors:\n" + "\n".join(errors)
            if errors
            else "No session-level errors reported.",
            foreground="#9a3412" if errors else "#166534",
            wraplength=1050,
        )
        self.session_status.pack(anchor="w", pady=6)
        table = ttk.Frame(body)
        table.pack(fill="both", expand=True)
        columns = ("capture", "pace", "status", "imu", "emg", "rails", "warnings")
        self.tree = ttk.Treeview(
            table, columns=columns, show="headings", height=9, selectmode="browse"
        )
        for name, title, width in zip(
            columns,
            ("Capture", "Pace", "Capture status", "IMU lost", "EMG lost", "ADC limits", "Warnings"),
            (110, 85, 150, 105, 105, 90, 100),
            strict=True,
        ):
            self.tree.heading(name, text=title)
            self.tree.column(name, width=width, minwidth=65)
        scroll = ttk.Scrollbar(table, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        for index, row in enumerate(report["captures"]):
            self.tree.insert(
                "",
                "end",
                iid=str(index),
                values=(
                    ("Ref " if row["reference"] else "") + row["id"],
                    row["pace"] or "—",
                    "Complete" if row["complete"] else "Incomplete",
                    packet_text(row, "imu"),
                    packet_text(row, "emg"),
                    count_text(row["emg_rail_samples"]),
                    len(row["issues"]),
                ),
            )
        self.details = ScrolledText(body, height=10, wrap="word", font=("Segoe UI", 10))
        self.details.pack(fill="both", expand=True, pady=8)
        self.details.configure(state="disabled")
        self.tree.bind("<<TreeviewSelect>>", self._selected)
        if report["captures"]:
            self.tree.selection_set("0")
            self._selected()
        ttk.Label(
            body,
            text="Select a capture for details. — means not reported. Review warnings before deciding whether to repeat; capture completion does not verify task success.",
            wraplength=1050,
        ).pack(anchor="w", pady=4)
        if path:
            ttk.Label(body, text=f"Saved report: {path}", wraplength=1050).pack(anchor="w", pady=4)
        ttk.Button(body, text="Close report", command=self.window.destroy).pack(anchor="e", pady=6)

    def _selected(self, _event=None):
        selected = self.tree.selection()
        if not selected:
            return
        self.details.configure(state="normal")
        self.details.delete("1.0", "end")
        self.details.insert("end", capture_details(self.report["captures"][int(selected[0])]))
        self.details.configure(state="disabled")
