# Johan Sandred, Majd
"""Read live BLE sensors in a background loop, following LIMB-HT25's reader."""

from __future__ import annotations

import asyncio
import contextlib
import threading
import time
from collections import deque

from ble_packets import (
    HIGH_RATE_EMG_HEADER,
    HIGH_RATE_IMU_PACKET,
    SENSOR_UUIDS,
    BleRecorder,
)
from emg_protocol import EmgActivationEstimator
from imu_twist import DEFAULT_TWIST_AXIS
from sensor_fusion import DualImuArmEstimator


class BleSensorReader:
    """Process every sensor sample and share the latest control state with the scene."""

    def __init__(
        self,
        device: str = "LIMBServer",
        address: str = "",
        shoulder_twist_axis: str = DEFAULT_TWIST_AXIS,
        wrist_twist_axis: str = DEFAULT_TWIST_AXIS,
    ) -> None:
        self.device = device
        self.address = address
        self.error = "Scanning for BLE device"
        self.status = "Scanning"
        self.thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._task: asyncio.Task | None = None
        self.decoder = BleRecorder(None, sample_sink=self._samples)
        self.imu_estimator = DualImuArmEstimator(
            shoulder_twist_axis=shoulder_twist_axis, wrist_twist_axis=wrist_twist_axis
        )
        self.emg_estimator = EmgActivationEstimator()
        self._emg_period = 0.01
        self._imu_device_time: int | None = None
        self._previous_imu_time: int | None = None
        self._clear_state()

    def _clear_state(self) -> None:
        # Keep a small waveform buffer without opening another BLE connection.
        self._emg_history = deque(maxlen=800)
        self._imu_history = {role: deque(maxlen=300) for role in ("shoulder", "wrist")}
        self.sensors: dict = {}
        self.angles: dict | None = None
        self.imu_time = 0.0
        self.emg_time = 0.0
        self.emg_raw: int | None = None
        self.activation: float | None = None
        self._previous_imu_time = None
        self._previous_emg_sequence = None
        self.imu_estimator.reset_calibration()
        self.emg_estimator.reset()

    def start(self) -> None:
        # Fail clearly if Bluetooth support is missing before opening the camera.
        import bleak  # noqa: F401

        if self.thread is not None and self.thread.is_alive():
            return
        self._stop.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        already_stopping = self._stop.is_set()
        self._stop.set()
        loop, task = self._loop, self._task
        # A connected client exits its polling loop and disconnects normally.
        # Cancel a pending scan once, without cancelling disconnect cleanup later.
        if (
            not already_stopping and self.status != "Connected"
            and loop is not None and task is not None
        ):
            with contextlib.suppress(RuntimeError):
                loop.call_soon_threadsafe(task.cancel)
        if self.thread is not None:
            self.thread.join(timeout=timeout)

    def calibrate(self, reset_emg: bool = True, reference_angles: object = None) -> None:
        with self._lock:
            self.imu_estimator.reset_calibration(reference_angles)
            if reset_emg:
                self.emg_estimator.reset()
                self.activation = None
            self.angles = None

    def latest(self) -> dict:
        with self._lock:
            imu_fresh = self.imu_time > 0 and time.monotonic() - self.imu_time <= 0.5
            return {
                "imus": dict(self.sensors) if imu_fresh else {},
                "imu_angles": dict(self.angles) if imu_fresh and self.angles else None,
                "imu_time": self.imu_time,
                "emg_time": self.emg_time,
                "emg_raw": self.emg_raw,
                "emg_activation": self.activation,
            }

    def correct_rotation(self, camera_proxy: float, weight: float) -> dict | None:
        """Correct axial drift while the acquisition thread keeps processing packets."""
        with self._lock:
            if self.imu_time <= 0 or time.monotonic() - self.imu_time > 0.5:
                return None
            corrected = self.imu_estimator.correct_rotation(camera_proxy, weight)
            if corrected is not None and self.angles is not None:
                self.angles["shoulder_rotation_proxy"] = corrected
                return dict(self.angles)
        return None

    def calibrate_emg(self) -> None:
        """Take a fresh relaxed-muscle baseline without changing IMU alignment."""
        with self._lock:
            self.emg_estimator.reset()
            self.activation = None

    def notification(self, sensor: str, data: bytes | bytearray) -> None:
        """Reuse the recording decoder; no session or output files are opened."""
        if sensor == "emg":
            period = 0.01
            if len(data) >= HIGH_RATE_EMG_HEADER.size:
                flags, _sequence, _timestamp, period_us, count, _dropped = (
                    HIGH_RATE_EMG_HEADER.unpack_from(data)
                )
                if (
                    flags >> 8 == 1
                    and period_us > 0
                    and 1 <= count <= 40
                    and len(data) == HIGH_RATE_EMG_HEADER.size + count * 2
                ):
                    period = period_us / 1_000_000
            with self._lock:
                if period != self._emg_period:
                    self._emg_period = period
                    # Keep the existing half-second rest calibration at the sample rate.
                    self.emg_estimator = EmgActivationEstimator(max(2, round(0.5 / period)))
                    self.activation = None
        elif sensor == "imu":
            self._imu_device_time = None
            if len(data) == HIGH_RATE_IMU_PACKET.size:
                flags, _sequence, shoulder_time, wrist_time, *_values = HIGH_RATE_IMU_PACKET.unpack(
                    data
                )
                if flags >> 8 == 1:
                    self._imu_device_time = shoulder_time if flags & 1 else wrist_time
        self.decoder.handle(sensor, bytes(data))

    def calibrate_emg_strength(self) -> bool:
        with self._lock:
            started = self.emg_estimator.calibrate_strength()
            if started:
                self.activation = None
            return started

    def graph_data(self) -> dict:
        """Copy recent received samples for the UI; stale streams show no trace."""
        now = time.monotonic()
        with self._lock:
            emg_fresh = self.emg_time > 0 and now - self.emg_time <= 0.5
            imu_fresh = self.imu_time > 0 and now - self.imu_time <= 0.5
            return {
                "emg": list(self._emg_history) if emg_fresh else [],
                "emg_period": self._emg_period,
                "activation": self.activation if emg_fresh else None,
                "emg_calibration": self.emg_estimator.status,
                "emg_envelope": self.emg_estimator.envelope if emg_fresh else None,
                "emg_rest": self.emg_estimator.rest_envelope,
                "emg_reference": self.emg_estimator.reference,
                "emg_strong": self.emg_estimator.strong_envelope,
                "emg_threshold": self.emg_estimator.activation_threshold,
                "imus": {
                    role: list(samples) if imu_fresh else []
                    for role, samples in self._imu_history.items()
                },
                "health": self.decoder.packet_health(),
                "status": self.status,
                "error": self.error,
            }

    def _samples(self, sensor: str, _sequence: int, channels: list) -> None:
        now = time.monotonic()
        with self._lock:
            if sensor == "emg":
                if self._previous_emg_sequence is not None:
                    difference = (_sequence - self._previous_emg_sequence) & 0xFFFFFFFF
                    if difference == 0 or difference >= 0x80000000:
                        return  # Duplicate/reordered data is not a new muscle sample.
                    if difference > 1:
                        self.emg_estimator.sample_gap()
                self._previous_emg_sequence = _sequence
                values = channels[0] if channels else []
                if not values:
                    self.emg_time = 0.0
                    self.activation = None
                    return
                for value in values:
                    self.activation = self.emg_estimator.update(value, self._emg_period)
                self._emg_history.extend(values)
                self.emg_raw = int(values[-1])
                self.emg_time = now
            elif sensor == "imu":
                units = self.decoder.imu_units
                if units not in (("g", "dps"), ("mg", "mdps")):
                    self.error = "IMU units unknown; use the current sensor firmware"
                    self.sensors = {}
                    self.angles = None
                    self.imu_time = 0.0
                    return
                scale = 0.001 if units == ("mg", "mdps") else 1.0
                sensors = {}
                for role, samples in zip(("shoulder", "wrist"), channels, strict=False):
                    if not samples or len(samples[-1]) < 6:
                        continue
                    values = samples[-1]
                    sensors[role] = {
                        "connected": True,
                        "accel_g": dict(zip("xyz", (v * scale for v in values[:3]), strict=True)),
                        "gyro_dps": dict(zip("xyz", (v * scale for v in values[3:6]), strict=True)),
                    }
                dt = now - self.imu_time if self.imu_time else 0.01
                if self._imu_device_time is not None and self._previous_imu_time is not None:
                    dt = (self._imu_device_time - self._previous_imu_time) / 1_000_000
                if dt <= 0 or dt > 0.5:
                    self.imu_estimator.reset_calibration()
                    dt = 0.01
                self._previous_imu_time = self._imu_device_time
                self.sensors = sensors
                for role, history in self._imu_history.items():
                    if role not in sensors:
                        history.clear()
                        continue
                    sensor = sensors[role]
                    history.append(
                        tuple(sensor["accel_g"][axis] for axis in "xyz")
                        + tuple(sensor["gyro_dps"][axis] for axis in "xyz")
                    )
                self.angles = self.imu_estimator.update(sensors, dt)
                self.imu_time = now if sensors else 0.0

    def _run(self) -> None:
        try:
            asyncio.run(self._connect_loop())
        except asyncio.CancelledError:
            pass
        finally:
            self._loop = None
            self._task = None
            with self._lock:
                self._clear_state()
            self.status = "Stopped"

    async def _connect_loop(self) -> None:
        from bleak import BleakClient, BleakScanner

        self._loop = asyncio.get_running_loop()
        self._task = asyncio.current_task()
        while not self._stop.is_set():
            try:
                self.status = "Scanning"
                self.error = f"Looking for {self.device}"
                target = self.address or await BleakScanner.find_device_by_filter(
                    lambda device, advertisement: (
                        self.device == (advertisement.local_name or device.name)
                    ),
                    timeout=10.0,
                )
                if target is None:
                    raise RuntimeError(f"{self.device} not found")
                async with BleakClient(target) as client:
                    missing = []
                    for sensor in ("imu", "emg"):
                        try:
                            await client.start_notify(
                                SENSOR_UUIDS[sensor],
                                lambda _sender, data, sensor=sensor: self.notification(
                                    sensor, data
                                ),
                            )
                        except Exception as error:
                            missing.append(f"{sensor.upper()}: {error}")
                    if len(missing) == 2:
                        raise RuntimeError("; ".join(missing))
                    self.error = "; ".join(missing)
                    self.status = "Connected"
                    while client.is_connected and not self._stop.is_set():
                        await asyncio.sleep(0.1)
                    if not self._stop.is_set():
                        raise RuntimeError("Disconnected; reconnecting")
            except Exception as error:
                self.error = str(error)
                self.status = "Reconnecting"
            finally:
                with self._lock:
                    self._clear_state()
            if not self._stop.is_set():
                await asyncio.sleep(1.0)
