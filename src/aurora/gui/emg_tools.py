# Majd
"""Saved-label review, EMG experiments and explicit model selection."""

from __future__ import annotations

import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from aurora.gui.project_support import console_python
from aurora.json_utils import read_json_object


class EmgToolsMixin:
    def _review_selected_emg(self) -> None:
        selected = self.recordings_tree.selection()
        path = self.recording_tree_paths.get(selected[0]) if selected else None
        if path is None:
            messagebox.showinfo("Choose a recording", "Select trial.json or a raw EMG CSV.")
            return
        python = self.simulation_python or console_python(Path(sys.executable))
        self._start_program(
            "EMG label review",
            "review",
            [str(python), "-m", "aurora.gui.emg_review", "--file", str(path)],
            parallel=True,
            process_key="review:emg",
        )

    def _train_emg_dialog(self) -> None:
        self.workflow.show_tool("emg_train")

    def _select_emg_model(self) -> None:
        filename = filedialog.askopenfilename(
            title="Choose an EMG bundle model.json", filetypes=(("EMG model", "model.json"),)
        )
        if not filename:
            return

        def load(cancelled):
            from aurora.ml.emg_model import EmgModel
            from aurora.recording.emg_labels import digest

            model = EmgModel(Path(filename).parent, require_review=False)
            evaluation = model.manifest["evaluation"]
            training_path = (model.directory / evaluation["training_file"]).resolve()
            training = read_json_object(training_path)
            if training.get("status") not in ("complete", "completed") or evaluation.get(
                "training_sha256"
            ) != digest(training_path):
                raise ValueError("The model needs a matching completed participant evaluation.")
            candidate = evaluation["candidate"]
            summary = [
                f"Model: {candidate}",
                f"Personal calibration trials: {training['personal_calibration_trials']}",
                "",
                "Held-out participant macro F1:",
            ]
            summary += [
                f"{fold['subject_id']}: {fold['models'][candidate]['phase']['macro_f1']:.3f}"
                for fold in training["folds"]
            ]
            summary += [
                "",
                "Check confusion matrices, unmatched phases and state delays in training.json.",
                "These offline scores do not establish live control accuracy.",
                "Live use still requires relaxed and strong-grip calibration.",
            ]
            return model, training_path, candidate, summary

        def ready(result, error):
            if error is not None:
                messagebox.showerror("Cannot select this model", str(error))
                return
            self._show_emg_model(*result)

        self.background_tasks.submit("emg_model", load, ready)

    def _show_emg_model(self, model, training_path, candidate, summary) -> None:
        dialog = tk.Toplevel(self)
        dialog.title("AURORA | Review EMG model")
        dialog.transient(self)
        ttk.Label(dialog, text="\n".join(summary), padding=15, wraplength=520).pack()
        ttk.Button(
            dialog, text="Open full results", command=lambda: self._open_existing(training_path)
        ).pack(pady=5)
        python = self.simulation_python or console_python(Path(sys.executable))
        ttk.Button(
            dialog,
            text="Plot participant scores",
            command=lambda: self._start_program(
                "EMG evaluation plot",
                "plot",
                [
                    str(python),
                    "-m",
                    "aurora.ml.emg_results",
                    "--training",
                    str(training_path),
                    "--show",
                ],
                parallel=True,
                process_key="plot:emg-results",
            ),
        ).pack(pady=5)

        def select():
            from aurora.json_utils import write_json
            from aurora.time_utils import utc_now

            try:
                model.manifest.update(reviewed_for_live=True, reviewed_utc=utc_now())
                write_json(model.directory / "model.json", model.manifest)
            except OSError as error:
                messagebox.showerror("Could not save selection", str(error), parent=dialog)
                return
            self.emg_model_path = model.directory
            self.emg_model_label.configure(text=f"EMG model: {candidate}")
            self.workflow.action_status.set(
                f"Selected EMG model for the next live session: {candidate}. Rest and grip calibration are still required."
            )
            self.fusion_grip_source.set("EMG")
            dialog.destroy()

        ttk.Button(dialog, text="Use this model for live EMG", command=select).pack(pady=10)

    def _use_amplitude_emg(self) -> None:
        self.emg_model_path = None
        self.emg_model_label.configure(text="EMG model: calibrated amplitude (default)")
        self.workflow.action_status.set(
            "Next live session uses calibrated EMG amplitude. Complete rest and deliberate grip calibration before control."
        )
