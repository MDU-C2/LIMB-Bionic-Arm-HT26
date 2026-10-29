"""One set of desktop session inputs, durable preferences and transient results."""

from __future__ import annotations

import json
import tkinter as tk
from pathlib import Path

from aurora.json_utils import write_json
from aurora.paths import REPOSITORY_ROOT
from aurora.recording.cup_protocol import TRIAL_SECONDS
from aurora.sensors.settings import DEFAULT_BLE_DEVICE_NAME, DEFAULT_GRIP_SOURCE

# Existing view attributes are aliases to these variables, never separate copies.
FIELDS = {
    "recording_subject": "S01",
    "recording_camera_side": "left",
    "recording_ble_device": DEFAULT_BLE_DEVICE_NAME,
    "recording_output": "outputs/recordings",
    "recording_serial_port": "",
    "recording_serial_baud": "115200",
    "recording_camera_depth": False,
    "recording_pose_model": "Balanced",
    "recording_emg_format": "auto",
    "cup_duration": f"{TRIAL_SECONDS:g}",
    "cup_condition": "natural",
    "dynamic_simulation": False,
    "save_simulation_torque": True,
    "save_simulation_telemetry": False,
    "fusion_camera_weight": "0.25",
    "fusion_depth": True,
    "fusion_grip_source": DEFAULT_GRIP_SOURCE.upper(),
    "fusion_shoulder_twist_axis": "y",
    "fusion_wrist_twist_axis": "y",
    "table_width_cm": "60",
    "table_depth_cm": "60",
}
CHOICES = {
    "recording_camera_side": ("left", "right"),
    "recording_pose_model": ("Fast", "Balanced", "Detailed"),
    "recording_emg_format": ("auto", "legacy-fixed", "legacy-prefix"),
    "cup_condition": ("natural", "slow", "fast"),
    "fusion_grip_source": ("EMG", "Camera", "Auto", "CAMERA", "AUTO"),
    "fusion_shoulder_twist_axis": ("x", "y", "z", "-x", "-y", "-z"),
    "fusion_wrist_twist_axis": ("x", "y", "z", "-x", "-y", "-z"),
}


class CupViewState:
    """A view of recorder events. The recorder owns readiness and every timer."""

    def __init__(self):
        self.status = dict.fromkeys(
            ("ready", "recording", "streams_ready", "can_start", "can_prepare", "can_record"), False
        )
        self.protocol = {"phase": "ready"}
        self.pending = False

    def update(self, data):
        self.status = (
            dict.fromkeys(
                ("ready", "recording", "streams_ready", "can_start", "can_prepare", "can_record"),
                False,
            )
            | data
        )
        if "protocol" in data:
            self.protocol = dict(data["protocol"])
            self.status.pop("protocol", None)
        self.pending = False

    @property
    def phase(self):
        return "arming" if self.pending else self.protocol.get("phase", "ready")

    @phase.setter
    def phase(self, value):
        self.pending = value == "arming"
        if not self.pending:
            self.protocol["phase"] = value

    def reset(self):
        self.__init__()


class ApplicationSession:
    def __init__(self, master, preferences=None):
        self.path = (
            Path(preferences) if preferences else REPOSITORY_ROOT / ".aurora/preferences.json"
        )
        values, self.preference_error = self.load(self.path)
        self.variables = {
            key: (tk.BooleanVar if isinstance(default, bool) else tk.StringVar)(
                master, value=values[key]
            )
            for key, default in FIELDS.items()
        }
        self.cup = CupViewState()
        self.selected_trial = None
        self.calibration_path = None
        self.selected_file = None
        self.last_output = tk.StringVar(master)
        self.review_root = tk.StringVar(master, value=values["recording_output"])
        self.context_locked = False
        self.notice = tk.StringVar(
            master,
            value="Preferences restored; connect and prepare this session. Saved preferences never certify calibration.",
        )
        self._previous = {key: variable.get() for key, variable in self.variables.items()}
        for key, variable in self.variables.items():
            variable.trace_add("write", lambda *_, key=key: self._changed(key))

    def _changed(self, key):
        variable = self.variables[key]
        editable = key in {"cup_duration", "cup_condition"} and self.cup.phase in {"ready", "rest"}
        value = variable.get()
        if value == self._previous[key]:
            return
        if self.context_locked and not editable and value != self._previous[key]:
            variable.set(self._previous[key])
            self.notice.set(
                "Finish the active session before changing its identity, devices, output or control settings."
            )
            return
        self._previous[key] = value
        if key in {"recording_subject", "recording_camera_side", "recording_ble_device"}:
            self.calibration_path = None
            self.cup.reset()
            self.notice.set(
                "Session identity changed. Connect and prepare the selected subject/arm; earlier trials remain available for review."
            )

    @staticmethod
    def load(path):
        values = dict(FIELDS)
        try:
            document = json.loads(Path(path).read_text(encoding="utf-8"))
            if document.get("format_version") != 1 or not isinstance(
                document.get("preferences"), dict
            ):
                raise ValueError("Unsupported preferences format")
            for key, default in FIELDS.items():
                value = document["preferences"].get(key, default)
                if (
                    type(value) is type(default)
                    and (not isinstance(value, str) or len(value) < 2048)
                    and (key not in CHOICES or value in CHOICES[key])
                ):
                    values[key] = value
        except FileNotFoundError:
            return values, ""
        except (OSError, ValueError, AttributeError) as error:
            return values, f"Preferences unavailable; defaults loaded: {error}"
        return values, ""

    def bind(self, gui):
        for name, variable in self.variables.items():
            setattr(gui, name, variable)

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        write_json(
            self.path,
            {
                "format_version": 1,
                "preferences": {key: value.get() for key, value in self.variables.items()},
            },
        )

    def close(self):
        """Release shared-variable callbacks on the UI thread before destroying Tk."""
        for variable in self.variables.values():
            for mode, callback in variable.trace_info():
                variable.trace_remove(mode, callback)

    def select_saved(self, path, *, reference=False):
        path = Path(path).resolve()
        manifest = path / "trial.json" if path.is_dir() else path
        self.selected_file = manifest
        self.last_output.set(str(path))
        if reference:
            self.calibration_path = manifest
        else:
            self.selected_trial = manifest
        self.notice.set(
            f"{'Reference' if reference else 'Trial'} saved: {path.name}. Review, replay or export it; collection can continue."
        )
