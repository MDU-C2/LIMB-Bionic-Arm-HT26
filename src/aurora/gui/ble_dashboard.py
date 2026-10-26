# Majd
"""Use the existing live BLE reader and graph window from the launcher."""

from __future__ import annotations

import time
import tkinter as tk
from tkinter import messagebox, simpledialog


class BleDashboardMixin:
    """Keep the live view connected until another program needs the sensors."""

    def _initialize_ble_dashboard(self) -> None:
        self.ble_reader = None
        self.ble_graphs = None
        self._ble_handoff_pending = False
        self.ble_sensor_connection = tk.StringVar(value="BLE: not connected")

    def open_ble_monitor(self) -> None:
        """Show live graphs without creating a recording or opening a COM port."""
        if self._ble_handoff_pending:
            return
        if any(process.key != "preview:record_oak_pose.py" for process in self._active_processes()):
            messagebox.showinfo(
                "Sensors in use",
                "Recording and live control use their own sensor connection. "
                "Stop the active program before opening the standalone BLE monitor.",
            )
            return
        if self.ble_reader is not None and self.ble_reader.thread.is_alive():
            self.ble_graphs.show()
            return
        device = self.recording_ble_device.get().strip()
        if not device:
            self.ble_sensor_connection.set("BLE: choose a device name first")
            return
        try:
            from aurora.gui.sensor_graphs import LiveSignalGraphs
            from aurora.sensors.ble_input import BleSensorReader

            self.ble_reader = BleSensorReader(device)
            self.ble_reader.start()
            if self.ble_graphs is None:
                self.ble_graphs = LiveSignalGraphs(
                    self.window, self._calibrate_ble_emg, self._calibrate_ble_emg_strength
                )
            else:
                self.ble_graphs.show()
        except (ImportError, OSError, RuntimeError, tk.TclError) as error:
            self._stop_ble_dashboard()
            self.ble_sensor_connection.set(f"BLE unavailable: {error}. Run Setup if needed.")
            self._append_log(f"[BLE monitor] {error}\n")
            return
        self.ble_sensor_connection.set(f"BLE: scanning for {device}")
        self._append_log(f"[BLE monitor] Connecting to {device}; no files are saved.\n")
        self._update_controls()

    def _calibrate_ble_emg(self) -> None:
        if self.ble_reader is not None:
            self.ble_reader.calibrate_emg()
            self._append_log("[BLE monitor] Keep the muscle relaxed for half a second.\n")

    def _poll_ble_dashboard(self) -> None:
        reader = self.ble_reader
        if reader is not None:
            data = reader.latest()
            imu_count = len(data["imus"])
            graph_data = reader.graph_data()
            emg_live = bool(graph_data["emg"])
            if reader.status == "Connected":
                text = f"BLE {self.recording_ble_device.get()}: {imu_count}/2 IMUs | "
                text += "EMG receiving" if emg_live else "EMG waiting for samples"
                if reader.error:
                    text += f" | {reader.error}"
            else:
                text = f"BLE {reader.status}: {reader.error}"
            self.ble_sensor_connection.set(
                "BLE: disconnecting before the next program starts"
                if self._ble_handoff_pending
                else text
            )
            if self.ble_graphs is not None:
                self.ble_graphs.update(reader)
        self.after(50, self._poll_ble_dashboard)

    def _calibrate_ble_emg_strength(self) -> None:
        if self.ble_reader is not None and self.ble_reader.calibrate_emg_strength():
            self._append_log("[BLE monitor] Hold a strong comfortable grip for 3 seconds.\n")

    def _stop_ble_dashboard(self) -> bool:
        """Release BLE before recording, firmware work, or live control starts."""
        if self.ble_reader is not None:
            self.ble_reader.stop(timeout=0)
            thread = self.ble_reader.thread
            if thread is not None and thread.is_alive():
                self.ble_sensor_connection.set("BLE: still disconnecting; try again shortly")
                return False
            self.ble_reader = None
        if self.ble_graphs is not None:
            self.ble_graphs.window.withdraw()
        self.ble_sensor_connection.set("BLE monitor stopped")
        return True

    def _release_ble_for(self, action) -> bool:
        """Schedule the action after Windows finishes releasing the preview connection."""
        if self._ble_handoff_pending:
            return True
        if self._stop_ble_dashboard():
            return False
        self._ble_handoff_pending = True
        deadline = time.monotonic() + 15.0

        def finish_disconnect() -> None:
            if not self._stop_ble_dashboard():
                if time.monotonic() < deadline:
                    self.after(100, finish_disconnect)
                    return
                self._ble_handoff_pending = False
                self.ble_sensor_connection.set(
                    "BLE disconnect timed out; stop the monitor and retry"
                )
                self._update_controls()
                return
            self._ble_handoff_pending = False
            self._update_controls()
            action()

        self._update_controls()
        self.after(100, finish_disconnect)
        return True

    def _edit_ble_device(self) -> None:
        if self._ble_handoff_pending or self._active_processes():
            messagebox.showinfo(
                "Sensors in use", "Stop the active program before changing devices."
            )
            return
        name = simpledialog.askstring(
            "BLE device name",
            "Name advertised by the ESP32. Normally AURORA.\n"
            "Changing this selects a device; it does not rename the board.",
            initialvalue=self.recording_ble_device.get(),
            parent=self.window,
        )
        if name is None or not name.strip():
            return

        def change_device() -> None:
            self.recording_ble_device.set(name.strip())
            self.open_ble_monitor()

        if not self._release_ble_for(change_device):
            change_device()
