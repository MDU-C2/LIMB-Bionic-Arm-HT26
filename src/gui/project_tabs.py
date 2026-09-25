"""Robot tooling and project-status pages for the AURORA GUI."""

from __future__ import annotations

import shutil
import tkinter as tk
from tkinter import messagebox, ttk

from project_support import (
    CARD,
    DOCUMENTATION_FILE,
    GUI_DOCUMENTATION_FILE,
    INK,
    INTERACTIVE_SIMULATION_SCRIPT,
    MOVEMENT_MODEL,
    MOVEMENT_RECOGNITION_SCRIPT,
    REPOSITORY_ROOT,
    discover_firmware_projects,
    display_path,
    environment_name,
    find_simulation_python,
    has_python_dependencies as _has_python_dependencies,
    serial_ports,
)


class ProjectTabsMixin:
    """Build robot/tooling pages and summarize repository readiness."""

    @staticmethod
    def _discover_firmware_projects():
        return discover_firmware_projects()

    def _build_robot_tab(self, tab: ttk.Frame) -> None:
        """Populate firmware build, flash, and serial-monitor controls."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "Build and flash the ESP32",
            "The firmware follows the ESP-IDF layout used by LIMB-HT25.",
        )
        card = self._card(
            tab,
            2,
            "Firmware target",
            "The GUI detects supported projects and serial ports; flashing "
            "always asks for confirmation.",
        )

        ttk.Label(card, text="Project", style="Card.TLabel").grid(row=2, column=0, sticky="w")
        project_row = ttk.Frame(card, style="Card.TFrame")
        project_row.grid(row=3, column=0, sticky="ew", pady=(4, 10))
        project_row.columnconfigure(0, weight=1)
        self.firmware_project_box = ttk.Combobox(
            project_row,
            textvariable=self.firmware_project,
            state="readonly",
        )
        self.firmware_project_box.grid(row=0, column=0, sticky="ew")
        ttk.Button(project_row, text="Refresh", command=self._refresh_firmware).grid(
            row=0, column=1, padx=(8, 0)
        )

        ttk.Label(card, text="Serial port", style="Card.TLabel").grid(
            row=4, column=0, sticky="w"
        )
        self.port_box = ttk.Combobox(card, textvariable=self.serial_port)
        self.port_box.grid(row=5, column=0, sticky="ew", pady=(4, 12))

        robot_actions = ttk.Frame(card, style="Card.TFrame")
        robot_actions.grid(row=6, column=0, sticky="w")
        self.build_button = ttk.Button(
            robot_actions,
            text="Build",
            style="Accent.TButton",
            command=lambda: self._run_firmware("build"),
        )
        self.build_button.pack(side="left", padx=(0, 8))
        self.flash_button = ttk.Button(
            robot_actions, text="Flash", command=lambda: self._run_firmware("flash")
        )
        self.flash_button.pack(side="left", padx=(0, 8))
        self.monitor_button = ttk.Button(
            robot_actions, text="Serial monitor", command=lambda: self._run_firmware("monitor")
        )
        self.monitor_button.pack(side="left")

        self.firmware_status = ttk.Label(
            tab, text="", style="PageSub.TLabel", wraplength=850, justify="left"
        )
        self.firmware_status.grid(row=3, column=0, sticky="w")

    def _build_info_tab(self, tab: ttk.Frame) -> None:
        """Populate project status and documentation shortcuts."""
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(2, weight=1)
        self._heading(
            tab,
            "Project information",
            "Check what is ready on this computer and open the main project documentation.",
        )
        info_card = ttk.Frame(tab, style="Card.TFrame", padding=16)
        info_card.grid(row=2, column=0, sticky="nsew", pady=(0, 12))
        info_card.columnconfigure(0, weight=1)
        info_card.rowconfigure(0, weight=1)
        self.info_text = tk.Text(
            info_card,
            height=16,
            wrap="word",
            state="disabled",
            relief="flat",
            bg=CARD,
            fg=INK,
            font=("Segoe UI", 10),
            padx=4,
            pady=4,
        )
        self.info_text.grid(row=0, column=0, sticky="nsew")

        actions = ttk.Frame(tab, style="Page.TFrame")
        actions.grid(row=3, column=0, sticky="w")
        ttk.Button(actions, text="Refresh status", command=self._refresh_all).pack(
            side="left", padx=(0, 8)
        )
        ttk.Button(
            actions,
            text="Open project README",
            command=lambda: self._open_existing(REPOSITORY_ROOT / "README.md"),
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            actions,
            text="Open simulation guide",
            command=lambda: self._open_existing(DOCUMENTATION_FILE),
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            actions,
            text="Open GUI guide",
            command=lambda: self._open_existing(GUI_DOCUMENTATION_FILE),
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            actions,
            text="Open docs folder",
            command=lambda: self._open_existing(REPOSITORY_ROOT / "docs"),
        ).pack(side="left")

    def _refresh_firmware(self) -> None:
        """Refresh firmware projects and currently visible serial ports."""
        previous = self.firmware_project.get()
        self.firmware_projects = self._discover_firmware_projects()
        values = list(self.firmware_projects)
        self.firmware_project_box.configure(values=values)
        if previous in self.firmware_projects:
            self.firmware_project.set(previous)
        elif values:
            self.firmware_project.set(values[0])
        else:
            self.firmware_project.set("")

        ports = serial_ports()
        current_port = self.serial_port.get()
        self.port_box.configure(values=ports)
        if not current_port and ports:
            self.serial_port.set(ports[0])
        self._firmware_selection_changed()

    def _firmware_selection_changed(self) -> None:
        """Update firmware details after the project selection changes."""
        project = self.firmware_projects.get(self.firmware_project.get())
        if project is None:
            self.firmware_status.configure(
                text="No ESP-IDF firmware project is present under firmware yet."
            )
        elif shutil.which(project.executable):
            self.firmware_status.configure(
                text=f"{project.system} tools ready. Project: {display_path(project.directory)}"
            )
        else:
            self.firmware_status.configure(
                text=(
                    f"{project.system} project found, but {project.executable} "
                    "is not available on PATH."
                )
            )
        self._update_controls()

    def _run_firmware(self, action: str) -> None:
        """Build, flash, or monitor the selected firmware project."""
        project = self.firmware_projects.get(self.firmware_project.get())
        if project is None:
            messagebox.showinfo("Firmware unavailable", "No supported firmware project was found.")
            return
        if shutil.which(project.executable) is None:
            messagebox.showerror(
                "Tool unavailable",
                f"{project.executable} is required for this {project.system} project.",
            )
            return

        port = self.serial_port.get().strip()
        if action in {"flash", "monitor"} and not port:
            messagebox.showerror("Serial port required", "Select or enter the robot serial port.")
            return
        if action == "flash" and not messagebox.askyesno(
            "Flash robot firmware",
            f"Flash {display_path(project.directory)} to {port}?",
        ):
            return

        if action in {"flash", "monitor"}:
            self._stop_serial_dashboard()

        command = ["idf.py"]
        if action in {"flash", "monitor"}:
            command.extend(["-p", port])
        command.append(action)

        self._start_program(
            f"Firmware {action}",
            "firmware",
            command,
            working_directory=project.directory,
        )

    # Project status

    def _refresh_info(self) -> None:
        """Render current dependency, program, and tool availability."""
        simulation_ready = (
            INTERACTIVE_SIMULATION_SCRIPT.is_file()
            and self.simulation_python is not None
        )
        motion_ai_ready = (
            MOVEMENT_RECOGNITION_SCRIPT.is_file()
            and MOVEMENT_MODEL.is_file()
            and self.movement_ai_ready
        )
        self.motion_ai_status.set(
            "Model ready" if motion_ai_ready else "Model or ONNX Runtime missing"
        )
        active_processes = self._active_processes()
        active = ", ".join(item.name for item in active_processes) or "None"
        lines = [
            "PROJECT STATUS\n",
            f"Simulation: {'Ready' if simulation_ready else 'Missing files or environment'}",
            f"Simulation environment: {environment_name(self.simulation_python)}",
            "NumPy, SciPy, PyBullet, and Pygame: "
            f"{'Ready' if self.simulation_python else 'Missing'}",
            f"Movement AI: {'Ready' if motion_ai_ready else 'Missing model or ONNX Runtime'}",
            f"Recording programs: {len(self.recording_programs)} found",
            f"Firmware projects: {len(self.firmware_projects)} found",
            f"ESP-IDF command: {'Available' if shutil.which('idf.py') else 'Not found'}",
            f"Active programs: {active}",
            "",
            "PORTABILITY\n",
            "Paths inside the repository are stored relative to the project root. "
            "The GUI can run from any clone location without path edits.",
            "",
            "SUPPORTED ENTRY POINTS\n",
            "Simulation: interactive task scene with keyboard or live camera + IMU control",
            "Motion AI: pose-recording playback and GRU profile recognition",
            "Recording: dual-IMU serial data and OAK-D pose capture",
            "Firmware: ESP-IDF CMake projects under firmware",
            "Recording settings are passed to record*.py and capture*.py programs",
        ]
        self.info_text.configure(state="normal")
        self.info_text.delete("1.0", "end")
        self.info_text.insert("1.0", "\n".join(lines))
        self.info_text.configure(state="disabled")

    def _refresh_all(self) -> None:
        """Refresh every discovery-backed area of the launcher."""
        self.simulation_python = find_simulation_python()
        self.movement_ai_ready = bool(
            self.simulation_python
            and _has_python_dependencies(self.simulation_python, ("numpy", "onnxruntime"))
        )
        self._refresh_recording_programs()
        self._refresh_recording_ports()
        self._refresh_recordings()
        self._refresh_firmware()
        self._refresh_info()
        if self.simulation_python is None and not self._active_processes():
            self._set_status("Simulation environment missing", "warning")
        elif not self._active_processes():
            self._set_status("Ready", "ready")
        self._append_log("Project status refreshed.\n")
