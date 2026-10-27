# Majd, Christian
"""USB diagnostic monitor and port discovery."""

from __future__ import annotations

import queue
import time

from aurora.gui.project_support import serial_ports
from aurora.gui.sensor_window import ImuMonitorWindow
from aurora.sensors.emg_protocol import extract_emg
from aurora.sensors.imu_protocol import extract_imus


class SerialDashboardMixin:
    """Own the optional USB sensor dashboard."""

    def _refresh_recording_ports(self) -> None:
        def ready(ports, error):
            if error is not None:
                self._set_serial_connection(f"Port scan failed: {error}")
                return
            self._apply_serial_ports(ports)

        self.background_tasks.submit("serial_ports", lambda _cancelled: serial_ports(), ready)

    def _apply_serial_ports(self, ports) -> None:
        workflow = getattr(self, "workflow", None)
        if workflow is not None and hasattr(workflow, "usb_ports"):
            workflow.usb_ports.configure(values=ports)
        if hasattr(self, "recording_port_box"):
            self.recording_port_box.configure(values=ports)
        if hasattr(self, "fusion_port_box"):
            self.fusion_port_box.configure(values=ports)
        if hasattr(self, "port_box"):
            self.port_box.configure(values=ports)
        selected_recording_port = self.recording_serial_port.get().strip()
        if selected_recording_port not in ports:
            self.recording_serial_port.set(ports[0] if ports else "")
        if hasattr(self, "serial_port"):
            selected_firmware_port = self.serial_port.get().strip()
            if selected_firmware_port not in ports:
                self.serial_port.set(ports[0] if ports else "")
        self._known_serial_ports = tuple(ports)

    def _poll_serial_ports(self) -> None:
        """Keep ESP32 port selectors current when hardware is hot-plugged."""
        if "serial_ports" not in self.background_tasks.jobs:
            self._refresh_recording_ports()
        self.after(1500, self._poll_serial_ports)

    def _ensure_imu_monitor(self) -> ImuMonitorWindow:
        if self.imu_monitor is None or not self.imu_monitor.exists:
            self.imu_monitor = ImuMonitorWindow(
                self.window,
                lambda: self.start_serial_dashboard(show_window=False),
                self._stop_serial_dashboard,
            )
            if self.latest_serial_sensor_packet is not None:
                self.imu_monitor.show_packet(self.latest_serial_sensor_packet)
        return self.imu_monitor

    def open_sensor_window(self) -> None:
        """Open the dedicated IMU/EMG monitor and connect if needed."""
        monitor = self._ensure_imu_monitor()
        monitor.focus()
        if not self.serial_sensor_reader.running:
            self.start_serial_dashboard(show_window=False)

    def _set_serial_connection(self, text: str) -> None:
        self.serial_sensor_connection.set(text)
        if self.imu_monitor is not None and self.imu_monitor.exists:
            self.imu_monitor.set_connection(text)

    def start_serial_dashboard(self, show_window: bool = True) -> None:
        """Read the selected ESP32 port and send packets to the sensor window."""
        if getattr(self, "_serial_handoff_pending", False) or any(
            process.kind in {"recording", "simulation", "firmware"}
            for process in self._active_processes()
        ):
            self._set_serial_connection(
                "Serial monitor paused while another program uses the sensors"
            )
            return
        if show_window:
            self._ensure_imu_monitor().focus()
        port = self.recording_serial_port.get().strip()
        if not port:
            self._refresh_recording_ports()
            port = self.recording_serial_port.get().strip()
        if not port:
            self._set_serial_connection("No serial port detected")
            return
        try:
            baud = int(self.recording_serial_baud.get().strip() or "115200")
            if baud <= 0:
                raise ValueError("Baud must be positive.")
        except ValueError as error:
            self._set_serial_connection(f"Invalid serial settings: {error}")
            return

        if not self._stop_serial_dashboard():
            self._release_serial_for(lambda: self.start_serial_dashboard(show_window=show_window))
            return
        self._set_serial_connection(f"Connecting to {port} at {baud} baud...")
        try:
            self.serial_sensor_reader.start(port, baud)
        except (OSError, RuntimeError) as error:
            self._set_serial_connection(f"Could not start serial monitor: {error}")

    def _stop_serial_dashboard(self) -> bool:
        """Release the COM port so firmware and recording tools can use it."""
        if not self.serial_sensor_reader.stop(timeout=0):
            self._set_serial_connection("Serial connection is stopping; wait for release")
            return False
        for _ in range(512):
            try:
                self.serial_sensor_reader.events.get_nowait()
            except queue.Empty:
                break
        if getattr(self, "latest_serial_sensor_packet", None) is not None:
            self.serial_imu_decoder.reset()
            self.latest_serial_sensor_packet = None
            if self.imu_monitor is not None and self.imu_monitor.exists:
                self.imu_monitor.show_packet(
                    {
                        "connection_generation": self.serial_imu_decoder.connection_generation,
                        "host_monotonic_ns": time.monotonic_ns(),
                        "imus": {},
                    }
                )
        self._set_serial_connection("Serial connection stopped")
        return True

    def _release_serial_for(self, action) -> bool:
        """Run a launch only after the previous serial owner has stopped."""
        if getattr(self, "_serial_handoff_pending", False):
            return False
        self._serial_handoff_pending = True
        deadline = time.monotonic() + 15.0

        def wait_for_release():
            if not self._stop_serial_dashboard():
                if time.monotonic() < deadline:
                    self.after(100, wait_for_release)
                    return
                self._serial_handoff_pending = False
                self._set_serial_connection("Serial release timed out; stop the monitor and retry")
                return
            self._serial_handoff_pending = False
            action()

        self.after(100, wait_for_release)
        return True

    def _drain_serial_sensor_events(self) -> None:
        """Apply background serial events to Tk widgets on the UI thread."""
        try:
            latest_packet = None
            for _ in range(512):
                event, value = self.serial_sensor_reader.events.get_nowait()
                if event == "connected":
                    port, baud, generation = value
                    self._serial_connection_changed(generation)
                    self._set_serial_connection(
                        f"Connected to {port} at {baud} baud | waiting for sensor data"
                    )
                elif event == "error":
                    self._set_serial_connection(
                        f"Could not open {self.serial_sensor_reader.port}: {value} | retrying"
                    )
                elif event == "fatal":
                    self._set_serial_connection(str(value))
                elif event == "line":
                    self._serial_connection_changed(value.connection_generation)
                    receipt_ns = value.host_monotonic_ns
                    packet = self.serial_imu_decoder.feed(value.text, host_monotonic_ns=receipt_ns)
                    if packet is not None:
                        packet["host_monotonic_ns"] = receipt_ns
                        packet["connection_generation"] = (
                            self.serial_imu_decoder.connection_generation
                        )
                        if self.imu_monitor is not None and self.imu_monitor.exists:
                            self.imu_monitor.accept_packet(packet)
                        latest_packet = packet
                elif event == "stopped" and not self.serial_sensor_reader.running:
                    self._set_serial_connection("Serial connection stopped")
        except queue.Empty:
            pass
        latest_packet = latest_packet or self.latest_serial_sensor_packet
        if latest_packet is not None:
            self._show_serial_sensor_packet(latest_packet)
        self.after(50, self._drain_serial_sensor_events)

    def _serial_connection_changed(self, generation):
        if generation == getattr(self, "_serial_reader_generation", None):
            return
        self._serial_reader_generation = generation
        self.serial_imu_decoder.reset()
        self.latest_serial_sensor_packet = None
        if self.imu_monitor is not None and self.imu_monitor.exists:
            self.imu_monitor.show_packet(
                {
                    "connection_generation": self.serial_imu_decoder.connection_generation,
                    "host_monotonic_ns": time.monotonic_ns(),
                    "imus": {},
                }
            )

    def _show_serial_sensor_packet(self, packet: dict[str, object]) -> None:
        """Forward one ESP32 packet to the separate monitor window."""
        sensors = extract_imus(packet)
        emg = extract_emg(packet)
        if not sensors and emg.get("connected") is not True:
            return
        if (
            any(sensor.get("connected") is True for sensor in sensors.values())
            or emg.get("connected") is True
        ):
            self.latest_serial_sensor_packet = packet
        fresh = time.monotonic_ns() - packet.get("host_monotonic_ns", 0) <= 500_000_000
        status = (
            f"Live data from {self.serial_sensor_reader.port} at {self.serial_sensor_reader.baud} baud"
            if fresh
            else "Serial: waiting for fresh sensor samples"
        )
        if self.serial_sensor_reader.queue_overruns:
            status += f" | Diagnostic queue drops: {self.serial_sensor_reader.queue_overruns}"
        self._set_serial_connection(status)
        if self.imu_monitor is not None and self.imu_monitor.exists:
            self.imu_monitor.show_packet(packet)
