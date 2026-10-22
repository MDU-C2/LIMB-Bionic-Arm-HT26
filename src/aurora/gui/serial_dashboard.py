# Majd, Christian
"""USB diagnostic monitor and port discovery."""

from __future__ import annotations

import queue

from aurora.gui.project_support import serial_ports
from aurora.gui.sensor_window import ImuMonitorWindow
from aurora.sensors.emg_protocol import extract_emg
from aurora.sensors.imu_protocol import extract_imus


class SerialDashboardMixin:
    """Own the optional USB sensor dashboard."""

    def _refresh_recording_ports(self) -> None:
        ports = serial_ports()
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
        ports = tuple(serial_ports())
        if ports != self._known_serial_ports:
            self._refresh_recording_ports()
            if ports:
                self._append_log(f"Serial ports detected: {', '.join(ports)}\n")
            else:
                self.serial_sensor_reader.stop()
                self._set_serial_connection("No serial port detected")
            self._update_controls()
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

        self._set_serial_connection(f"Connecting to {port} at {baud} baud...")
        self.serial_imu_decoder.reset()
        self.serial_sensor_reader.start(port, baud)

    def _stop_serial_dashboard(self) -> None:
        """Release the COM port so firmware and recording tools can use it."""
        self.serial_sensor_reader.stop()
        self._set_serial_connection("Serial connection stopped")

    def _drain_serial_sensor_events(self) -> None:
        """Apply background serial events to Tk widgets on the UI thread."""
        try:
            while True:
                event, value = self.serial_sensor_reader.events.get_nowait()
                if event == "connected":
                    port, baud = value
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
                    packet = self.serial_imu_decoder.feed(str(value))
                    if packet is not None:
                        self._show_serial_sensor_packet(packet)
                elif event == "stopped" and not self.serial_sensor_reader.running:
                    self._set_serial_connection("Serial connection stopped")
        except queue.Empty:
            pass
        self.after(50, self._drain_serial_sensor_events)

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
        self._set_serial_connection(
            f"Live data from {self.serial_sensor_reader.port} at "
            f"{self.serial_sensor_reader.baud} baud"
        )
        if self.imu_monitor is not None and self.imu_monitor.exists:
            self.imu_monitor.show_packet(packet)
