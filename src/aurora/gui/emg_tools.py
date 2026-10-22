# Majd
"""Saved-label review, EMG experiments and explicit model selection."""

from __future__ import annotations

import json
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from aurora.gui.project_support import console_python


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
        directory = filedialog.askdirectory(
            title="Choose the folder containing reviewed EMG labels"
        )
        if not directory:
            return
        dialog = tk.Toplevel(self)
        dialog.title("AURORA | Compare EMG models")
        dialog.transient(self)
        fields = {}
        rows = (
            ("Models", "lda ann lstm"),
            ("Feature sets", "basic full"),
            ("Epochs", "30"),
            ("Sequence windows", "4"),
            ("Personal calibration trials", "0"),
        )
        for row, (title, default) in enumerate(rows):
            ttk.Label(dialog, text=title).grid(row=row, column=0, sticky="w", padx=10, pady=6)
            fields[title] = tk.StringVar(dialog, value=default)
            ttk.Entry(dialog, textvariable=fields[title], width=24).grid(row=row, column=1, padx=10)
        ttk.Label(
            dialog,
            text="Needs reviewed labels from at least two subjects.\n"
            "Calibration uses separate held-out-subject trials.\n"
            "Results and bundles go to outputs/training. No model is selected automatically.",
            wraplength=450,
        ).grid(row=len(rows), column=0, columnspan=2, padx=10, pady=10)

        def start():
            try:
                epochs = int(fields["Epochs"].get())
                length = int(fields["Sequence windows"].get())
                calibration = int(fields["Personal calibration trials"].get())
                models = fields["Models"].get().split()
                features = fields["Feature sets"].get().split()
                if not (1 <= epochs <= 10000 and 1 <= length <= 20 and calibration >= 0):
                    raise ValueError("Check epochs, sequence windows and calibration trials.")
                if not models or not set(models) <= {"lda", "ann", "lstm"}:
                    raise ValueError("Models: lda, ann, lstm.")
                if not features or not set(features) <= {"basic", "full"}:
                    raise ValueError("Feature sets: basic, full.")
            except ValueError as error:
                messagebox.showerror("Check training settings", str(error), parent=dialog)
                return
            python = self.simulation_python or console_python(Path(sys.executable))
            self._start_program(
                "EMG participant evaluation",
                "training",
                [
                    str(python),
                    "-u",
                    "-m",
                    "aurora.ml.train_emg",
                    "--data",
                    directory,
                    "--models",
                    *models,
                    "--features",
                    *features,
                    "--epochs",
                    str(epochs),
                    "--sequence-length",
                    str(length),
                    "--calibration-trials",
                    str(calibration),
                ],
                parallel=True,
                process_key="training:emg",
            )
            dialog.destroy()

        ttk.Button(dialog, text="Start comparison", command=start).grid(
            row=len(rows) + 1, column=0, columnspan=2, pady=10
        )

    def _select_emg_model(self) -> None:
        filename = filedialog.askopenfilename(
            title="Choose an EMG bundle model.json", filetypes=(("EMG model", "model.json"),)
        )
        if not filename:
            return
        from aurora.ml.emg_model import EmgModel
        from aurora.recording.emg_labels import digest

        try:
            model = EmgModel(Path(filename).parent, require_review=False)
            evaluation = model.manifest["evaluation"]
            training_path = (model.directory / evaluation["training_file"]).resolve()
            training = json.loads(training_path.read_text(encoding="utf-8"))
            if training.get("status") != "completed" or evaluation.get("training_sha256") != digest(
                training_path
            ):
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
                "Check confusion matrices, missed transitions and delays in training.json.",
                "These offline scores do not establish live control accuracy.",
                "Live use still requires relaxed and strong-grip calibration.",
            ]
        except (OSError, ValueError, KeyError, TypeError, ImportError) as error:
            messagebox.showerror("Cannot select this model", str(error))
            return
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
            self.fusion_grip_source.set("EMG")
            dialog.destroy()

        ttk.Button(dialog, text="Use this model for live EMG", command=select).pack(pady=10)

    def _use_amplitude_emg(self) -> None:
        self.emg_model_path = None
        self.emg_model_label.configure(text="EMG model: calibrated amplitude (default)")
