# Majd, Christian
"""Set up recording sessions and start the selected sources."""

from __future__ import annotations

import math
import sys
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

from aurora.gui.cup_recording import CupRecordingMixin
from aurora.gui.project_support import (
    RECORDER_DETAILS,
    console_python,
    discover_recording_programs,
    display_path,
    project_path,
)
from aurora.gui.recording_browser import RecordingBrowserMixin
from aurora.gui.recording_settings import BLE_SENSOR_OPTIONS as BLE_SENSOR_OPTIONS
from aurora.gui.recording_settings import DEFAULT_BATCH_SOURCES as DEFAULT_BATCH_SOURCES
from aurora.gui.recording_settings import DEFAULT_BLE_SENSORS as DEFAULT_BLE_SENSORS
from aurora.gui.recording_settings import MAINTAINED_SOURCES as MAINTAINED_SOURCES
from aurora.gui.recording_settings import batch_arguments as batch_arguments
from aurora.gui.recording_views import RecordingViewsMixin
from aurora.gui.sensor_window import ImuMonitorWindow
from aurora.gui.serial_dashboard import SerialDashboardMixin
from aurora.gui.serial_sensor import SerialSensorReader
from aurora.paths import PACKAGE_DIRECTORY
from aurora.recording.common import STOP_COMMAND
from aurora.recording.emg_trials import GripTrial
from aurora.sensors.imu_protocol import ImuStreamDecoder


class RecordingTabMixin(
    CupRecordingMixin, RecordingViewsMixin, RecordingBrowserMixin, SerialDashboardMixin
):
    """Provide recording configuration, launch actions, and file browsing."""

    def _initialize_recording_state(self) -> None:
        self.recording_program = tk.StringVar()
        self.recording_trial_id = tk.StringVar(value="T001")
        self.recording_test_type = tk.StringVar(value="movement_imu")
        self.recording_duration = tk.StringVar(value="0")
        self.recording_ble_sensors = {
            name: tk.BooleanVar(value=name in DEFAULT_BLE_SENSORS)
            for name, _label in BLE_SENSOR_OPTIONS
        }
        self.recording_ble_dataset = tk.BooleanVar(value=False)
        self.recording_grip_trial = tk.BooleanVar(value=False)
        self.recording_grip_repeats = tk.StringVar(value="5")
        self.recording_grip_status = tk.StringVar(
            value="The operator relays the grip cues to the subject."
        )
        self.recording_movement_label = tk.StringVar(value="1")
        self.serial_sensor_reader = SerialSensorReader()
        self.serial_imu_decoder = ImuStreamDecoder()
        self.latest_serial_sensor_packet: dict[str, object] | None = None
        self.serial_sensor_connection = tk.StringVar(value="Waiting for a serial port")
        self.imu_monitor: ImuMonitorWindow | None = None
        self.recording_sources = {
            filename: tk.BooleanVar(value=filename in DEFAULT_BATCH_SOURCES)
            for filename, _title, _description in MAINTAINED_SOURCES
        }
        self.recordings_root = self.session.review_root
        self.recordings_filter = tk.StringVar()

        self.recording_programs: dict[str, Path] = {}
        self.recording_files: list[Path] = []
        self.recording_metadata: dict[Path, tuple[int, float]] = {}
        self.recording_tree_paths: dict[str, Path] = {}
        self.recordings_scan_error = ""
        self.recordings_scan_truncated = False
        self.recordings_filter_after: str | None = None
        self._known_serial_ports: tuple[str, ...] = ()
        self._initialize_cup_recording_state()

    def _bind_recording_traces(self) -> None:
        for variable in (
            self.recording_subject,
            self.recording_camera_side,
            self.recording_ble_device,
        ):
            variable.trace_add("write", lambda *_: self.after_idle(self._refresh_cup_guidance))
        self.recording_program.trace_add("write", lambda *_: self._recording_selection_changed())
        self.recordings_filter.trace_add("write", lambda *_: self._schedule_recordings_filter())
        for variable in self.recording_sources.values():
            variable.trace_add("write", lambda *_: self._update_controls())
        for variable in self.recording_ble_sensors.values():
            variable.trace_add("write", lambda *_: self._update_controls())

    def _recording_selection_changed(self) -> None:
        program = self.recording_programs.get(self.recording_program.get())
        if program is None:
            text = "No recording program is available."
        else:
            text = RECORDER_DETAILS.get(
                program.name,
                "Discovered repository entry point. Check its --help before running it.",
            )
        if hasattr(self, "recording_program_status"):
            self.recording_program_status.configure(text=text)
        self._update_controls()

    def open_camera_monitor(self) -> None:
        """Open the existing OAK-D pose preview without starting a recording."""
        self.start_sensor_preview("record_oak_pose.py")

    def _refresh_recording_programs(self) -> None:
        previous = self.recording_program.get()
        labels = {
            "record_ble_sensors.py": "Raw Bluetooth signals",
            "record_oak_pose.py": "Camera video and pose",
            "record_high_rate_serial.py": "USB signals with sample export",
            "record_serial_sensors.py": "Raw USB messages",
            "record_cup_trials.py": "Guided cup transfers",
            "capture_movement.py": "Camera movement profiles",
        }
        self.recording_programs = {
            labels[path.name]: path
            for path in discover_recording_programs().values()
            if path.name in labels
        }
        values = list(self.recording_programs)
        if hasattr(self, "recording_program_box"):
            self.recording_program_box.configure(values=values)
        if previous in self.recording_programs:
            self.recording_program.set(previous)
        elif values:
            self.recording_program.set(values[0])
        else:
            self.recording_program.set("")

        available_names = {path.name for path in self.recording_programs.values()}
        for filename, button in getattr(self, "recording_source_buttons", {}).items():
            button.configure(state="normal" if filename in available_names else "disabled")
            if filename not in available_names:
                self.recording_sources[filename].set(False)
        self._recording_selection_changed()

    def _selected_ble_sensors(self) -> tuple[str, ...]:
        """Return BLE streams selected for the next recording."""
        return tuple(
            name for name, _label in BLE_SENSOR_OPTIONS if self.recording_ble_sensors[name].get()
        )

    def _recording_setup(self, filenames: list[str]) -> tuple[dict[str, str], list[str]] | None:
        output_value = self.recording_output.get().strip()
        if not output_value:
            messagebox.showerror("Output folder required", "Choose a recording output folder.")
            return None
        try:
            duration = float(self.recording_duration.get().strip())
            if not math.isfinite(duration) or duration < 0:
                raise ValueError("Duration must be finite and 0 or more seconds.")
            if self.recording_grip_trial.get():
                if (
                    "record_ble_sensors.py" not in filenames
                    or "emg" not in self._selected_ble_sensors()
                ):
                    raise ValueError("Select Bluetooth and EMG for a grip trial.")
                repeats = int(self.recording_grip_repeats.get())
                if self.recording_ble_dataset.get() or not 1 <= repeats <= 20:
                    raise ValueError("Choose 1 to 20 grip repeats and leave movement labels off.")
                minimum_duration = GripTrial(repeats).duration + 3
                if duration and duration < minimum_duration:
                    raise ValueError(f"Use duration 0 or at least {minimum_duration:g} seconds.")
            baud = int(self.recording_serial_baud.get().strip() or "115200")
            if baud <= 0:
                raise ValueError("Baud must be positive.")
            output = project_path(output_value)
            output.mkdir(parents=True, exist_ok=True)
            camera_arguments = self._camera_arguments()
        except (OSError, ValueError) as error:
            messagebox.showerror("Could not prepare recording", str(error))
            return None

        serial_recorders = {"record_serial_sensors.py", "record_high_rate_serial.py"}
        if (
            serial_recorders.intersection(filenames)
            and not self.recording_serial_port.get().strip()
        ):
            messagebox.showerror("Serial port required", "Choose the sensor serial port first.")
            return None
        if "record_ble_sensors.py" in filenames and not self.recording_ble_device.get().strip():
            messagebox.showerror("BLE device required", "Enter the AURORA BLE device name first.")
            return None
        if "record_ble_sensors.py" in filenames and not self._selected_ble_sensors():
            messagebox.showerror(
                "Bluetooth sensor required",
                "Select at least one Bluetooth sensor stream (IMU or EMG).",
            )
            return None
        if (
            "record_ble_sensors.py" in filenames
            and self.recording_ble_dataset.get()
            and not self.recording_movement_label.get().strip()
        ):
            messagebox.showerror("Movement label required", "Enter a movement label first.")
            return None

        batch_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        environment = {
            "AURORA_OUTPUT_DIR": str(output.resolve()),
            "AURORA_SUBJECT": self.recording_subject.get().strip() or "session",
            "AURORA_DURATION_SECONDS": str(duration),
            "AURORA_CAMERA_SIDE": self.recording_camera_side.get(),
            "AURORA_BLE_DEVICE": self.recording_ble_device.get().strip(),
            "AURORA_SERIAL_PORT": self.recording_serial_port.get().strip(),
            "AURORA_SERIAL_BAUD": str(baud),
            "AURORA_TRIAL_ID": self.recording_trial_id.get().strip(),
            "AURORA_TEST_TYPE": self.recording_test_type.get().strip(),
            "AURORA_SESSION_ID": batch_id,
            "AURORA_BATCH_SOURCES": ",".join(filenames),
        }
        return environment, camera_arguments

    def _recording_arguments(self, filename, camera_arguments):
        arguments = batch_arguments(
            filename,
            camera_arguments,
            self.recording_movement_label.get().strip() if self.recording_ble_dataset.get() else "",
            self._selected_ble_sensors(),
            self.recording_grip_trial.get(),
            int(self.recording_grip_repeats.get()) if self.recording_grip_trial.get() else 5,
        )
        if filename == "record_ble_sensors.py" and self.recording_emg_format.get() != "auto":
            arguments += ["--emg-format", self.recording_emg_format.get()]
        return arguments

    def start_selected_recordings(self) -> None:
        """Launch every checked maintained source as one recording batch."""
        filenames = [
            filename for filename, variable in self.recording_sources.items() if variable.get()
        ]
        if not filenames:
            messagebox.showinfo("Choose sources", "Select at least one recording source.")
            return
        if self._active_processes():
            messagebox.showinfo(
                "Programs running",
                "Stop the active program or recording batch before starting another.",
            )
            return
        setup = self._recording_setup(filenames)
        if setup is None:
            return
        environment, camera_arguments = setup
        if not self._stop_serial_dashboard():
            self._release_serial_for(self.start_selected_recordings)
            return
        if self._release_ble_for(self.start_selected_recordings):
            return
        python = self.simulation_python or console_python(Path(sys.executable))
        launched = 0
        for filename in filenames:
            program = PACKAGE_DIRECTORY / "recording" / filename
            if not program.is_file():
                self._append_log(f"[Recording batch] Missing source: {display_path(program)}\n")
                continue
            arguments = self._recording_arguments(filename, camera_arguments)
            display_name = next(
                title
                for candidate, title, _description in MAINTAINED_SOURCES
                if candidate == filename
            )
            if self._start_program(
                display_name,
                "recording",
                [str(python), "-u", str(program), *arguments],
                environment=environment,
                parallel=True,
                process_key=f"recording:{filename}",
                stop_input=STOP_COMMAND,
            ):
                launched += 1
        if launched:
            self._append_log(
                f"[Recording batch {environment['AURORA_SESSION_ID']}] "
                f"Started {launched} source(s).\n"
            )

    def start_single_recording(self) -> None:
        """Run one discovered recorder, primarily for training/experimental tools."""
        program = self.recording_programs.get(self.recording_program.get())
        if program is None:
            messagebox.showinfo("Recording unavailable", "No recording program was found in src.")
            return
        if program.name == "capture_movement.py":
            self.workflow.show_tool("movement_capture")
            return
        if program.name == "record_cup_trials.py":
            self.notebook.select(self.tabs["recording"])
            self.prepare_cup_recording()
            return
        if self._active_processes():
            messagebox.showinfo("Programs running", "Stop active programs first.")
            return
        setup = self._recording_setup([program.name])
        if setup is None:
            return
        environment, arguments = setup
        if self._release_ble_for(self.start_single_recording):
            return
        arguments = self._recording_arguments(program.name, arguments)
        python = self.simulation_python or console_python(Path(sys.executable))
        self._start_program(
            program.stem,
            "recording",
            [str(python), "-u", str(program), *arguments],
            environment=environment,
            parallel=True,
            process_key=f"recording:{program.name}",
            stop_input=STOP_COMMAND,
        )

    def stop_recordings(self) -> None:
        recordings = self._active_processes("recording")
        if recordings and self._confirm_sensitive_stop(recordings):
            self._stop_processes(recordings)

    def preview_selected_source(self) -> None:
        program = self.recording_programs.get(self.recording_program.get())
        if program is None or program.name not in RECORDER_DETAILS:
            messagebox.showinfo("Preview unavailable", "Select a maintained sensor source first.")
            return
        self.start_sensor_preview(program.name)

    def start_sensor_preview(self, filename: str) -> None:
        if filename not in RECORDER_DETAILS:
            return
        if filename == "record_ble_sensors.py":
            self.open_ble_monitor()
            return
        if filename in {"record_serial_sensors.py", "record_high_rate_serial.py"}:
            self.start_serial_dashboard()
            return
        non_preview = [process for process in self._active_processes() if process.kind != "preview"]
        camera_can_share = filename == "record_oak_pose.py" and all(
            process.name == "Interactive task simulator" for process in non_preview
        )
        if non_preview and not camera_can_share:
            messagebox.showinfo(
                "Program running",
                "Stop recording, simulation, or firmware tools before opening a preview.",
            )
            return
        program = PACKAGE_DIRECTORY / "recording" / filename
        if not program.is_file():
            messagebox.showerror("Preview unavailable", f"Program not found: {program}")
            return
        environment = {
            "AURORA_OUTPUT_DIR": str(project_path(self.recording_output.get()).resolve()),
            "AURORA_SUBJECT": self.recording_subject.get().strip() or "session",
            "AURORA_DURATION_SECONDS": self.recording_duration.get().strip() or "0",
            "AURORA_CAMERA_SIDE": self.recording_camera_side.get(),
            "AURORA_TRIAL_ID": self.recording_trial_id.get().strip(),
            "AURORA_TEST_TYPE": self.recording_test_type.get().strip(),
        }
        arguments = self._camera_arguments() if filename == "record_oak_pose.py" else []
        python = self.simulation_python or console_python(Path(sys.executable))
        self._start_program(
            "OAK-D camera",
            "preview",
            [str(python), "-u", str(program), "--preview", *arguments],
            environment=environment,
            parallel=True,
            process_key=f"preview:{filename}",
            stop_input=STOP_COMMAND,
        )

    def _camera_arguments(self):
        levels = {"Fast": 0, "Balanced": 1, "Detailed": 2}
        arguments = ["--pose-model", str(levels[self.recording_pose_model.get()])]
        if self.recording_camera_depth.get():
            arguments.append("--depth")
        return arguments
