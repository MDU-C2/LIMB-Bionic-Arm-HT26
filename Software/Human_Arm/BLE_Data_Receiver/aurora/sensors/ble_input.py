# Johan Sandred, Majd
"""Read BLE sensors in the background and share the latest values."""

from __future__ import annotations

import asyncio
import contextlib
import threading
import time
import uuid
from collections import deque
from pathlib import Path

from aurora.sensors.ble_discovery import find_device
from aurora.sensors.ble_protocol import (
    SENSOR_UUIDS,
    BleRecorder,
)
from aurora.sensors.clock_sync import BleClockSyncClient, ClockSynchronizer
from aurora.sensors.emg_protocol import EmgActivationEstimator
from aurora.sensors.fusion import DualImuArmEstimator
from aurora.sensors.imu_twist import DEFAULT_TWIST_AXIS
from aurora.sensors.settings import DEFAULT_BLE_DEVICE_NAME
from aurora.time_utils import utc_now


class BleSensorReader:
    """Process every sensor sample and share the latest control state with the scene."""

    def __init__(
        self,
        device: str = DEFAULT_BLE_DEVICE_NAME,
        address: str = "",
        shoulder_twist_axis: str = DEFAULT_TWIST_AXIS,
        wrist_twist_axis: str = DEFAULT_TWIST_AXIS,
        *,
        synchronize_clock: bool = False,
        emg_model: Path | None = None,
        clock=None,
        side: str = "left",
    ) -> None:
        self.device = device
        self.clock = clock or (lambda: time.monotonic())
        self.address = address
        self.error = "Scanning for BLE device"
        self.status = "Scanning"
        self.thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._task: asyncio.Task | None = None
        self.synchronize_clock = synchronize_clock
        self.clock_sync = ClockSynchronizer()
        self._recording_lock = threading.Lock()
        self._recording_idle = threading.Condition(self._recording_lock)
        self._recording_inflight = 0
        self._recording = None
        self._pending_writer = None
        self._recording_clock_epochs = []
        self.recording_error = ""
        self.connection_generation = 0
        self.mounting_session_id = uuid.uuid4().hex
        self.device_clock_resets = 0
        self.decoder = BleRecorder(None, sample_sink=self._samples)
        self.imu_estimator = DualImuArmEstimator(
            shoulder_twist_axis=shoulder_twist_axis, wrist_twist_axis=wrist_twist_axis, side=side
        )
        self.emg_estimator = EmgActivationEstimator()
        self.emg_classifier = None
        if emg_model is not None:
            from aurora.ml.emg_model import LiveEmgClassifier

            self.emg_classifier = LiveEmgClassifier(emg_model, background=True)
        self._emg_period = 0.01
        self._imu_device_time: int | None = None
        self._imu_device_times = {}
        self._previous_imu_time: int | None = None
        self.reference_capture = None
        self._clear_state()

    def _clear_state(self) -> None:
        if self.reference_capture is not None:
            self.reference_capture.error = "Connection changed; repeat mounting calibration"
            self.reference_capture = None
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
        self._previous_imu_times = {}
        self._imu_role_receipts = {}
        self.alignment_profile = None
        self._previous_emg_sequence = None
        self._previous_emg_device_time = None
        self._previous_emg_end_us = None
        self._previous_imu_sequence = None
        self.imu_motion_gap = False
        self.imu_invalid_reason = ""
        self.imu_estimator.clear_alignment()
        self.emg_estimator.reset()
        if self.emg_classifier is not None:
            self.emg_classifier.reset()

    def start(self) -> None:
        # Fail clearly if Bluetooth support is missing before opening the camera.
        import bleak  # noqa: F401

        if self.thread is not None and self.thread.is_alive():
            return
        if self.emg_classifier is not None:
            self.emg_classifier.reopen()
        self._stop.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self, timeout: float = 3.0) -> bool:
        already_stopping = self._stop.is_set()
        self._stop.set()
        loop, task = self._loop, self._task
        # A connected client exits its polling loop and disconnects normally.
        # Cancel a pending scan once, without cancelling disconnect cleanup later.
        if (
            not already_stopping
            and self.status != "Connected"
            and loop is not None
            and task is not None
        ):
            with contextlib.suppress(RuntimeError):
                loop.call_soon_threadsafe(task.cancel)
        if self.thread is not None:
            self.thread.join(timeout=timeout)
            if self.thread.is_alive():
                return False
        if self.emg_classifier is not None and not self.emg_classifier.close(timeout):
            return False
        if self._recording is not None or self._pending_writer is not None:
            if timeout <= 0:
                return False
            self.stop_recording()
        return True

    def invalidate_calibration(self) -> None:
        """Discard mounting and baseline validity without interrupting raw capture."""
        with self._lock:
            self.mounting_session_id = uuid.uuid4().hex
            self.alignment_profile = None
            self.imu_estimator.clear_alignment()
            self.emg_estimator.reset()
            self.activation = None
            if self.emg_classifier is not None:
                self.emg_classifier.reset()
            self.angles = None

    def calibrate(self, reset_emg: bool = True, reference_angles: object = None) -> None:
        with self._lock:
            self.imu_motion_gap = False
            self.imu_invalid_reason = ""
            self.imu_estimator.reset_calibration(reference_angles)
            if reset_emg:
                self.emg_estimator.reset()
                self.activation = None
                if self.emg_classifier is not None:
                    self.emg_classifier.reset()
            self.angles = None

    def latest(self) -> dict:
        with self._lock:
            now = self.clock()
            sensors = {
                role: dict(sensor)
                for role, sensor in self.sensors.items()
                if now - self._imu_role_receipts.get(role, self.imu_time) <= 0.5
            }
            imu_fresh = self.imu_time > 0 and now - self.imu_time <= 0.5
            from aurora.sensors.fusion import fresh_imu_angles

            return {
                "imus": sensors if imu_fresh else {},
                "imu_angles": fresh_imu_angles(self.angles, sensors) if imu_fresh else None,
                "imu_angle_reference": {
                    name: "human_reference"
                    if name in self.imu_estimator.referenced_channels()
                    else "change_since_zeroing"
                    for name in self.angles or {}
                },
                "imu_time": self.imu_time,
                "connection_generation": self.connection_generation,
                "mounting_session_id": self.mounting_session_id,
                "imu_motion_gap": self.imu_motion_gap,
                "imu_invalid_reason": self.imu_invalid_reason,
                "imu_calibrated": all(
                    role in self.imu_estimator.twist.bias for role in ("shoulder", "wrist")
                ),
                "emg_time": self.emg_time,
                "emg_raw": self.emg_raw,
                "emg_activation": self.activation,
                "emg_phase": self.emg_classifier.phase if self.emg_classifier else None,
                "emg_model_grip": self.emg_classifier.grip(self.activation)
                if self.emg_classifier
                else None,
            }

    def correct_angles(self, camera_angles: object, weight: float) -> dict | None:
        with self._lock:
            if self.imu_time <= 0 or self.clock() - self.imu_time > 0.5:
                return None
            self.angles = self.imu_estimator.correct_angles(camera_angles, weight) or self.angles
            return dict(self.angles) if self.angles else None

    def correct_rotation(self, camera_proxy: float, weight: float) -> dict | None:
        """Correct axial drift while the acquisition thread keeps processing packets."""
        with self._lock:
            if self.imu_time <= 0 or self.clock() - self.imu_time > 0.5:
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
            if self.emg_classifier is not None:
                self.emg_classifier.reset()

    def notification(self, sensor: str, data: bytes | bytearray) -> None:
        """Decode each packet and save it when a trial is active."""
        receipt_ns = time.monotonic_ns()
        receipt_utc = utc_now()
        payload = bytes(data)
        with self._recording_lock:
            writer = self._recording
            if writer is not None:
                self._recording_inflight += 1
        try:
            self.decoder.handle(
                sensor, payload, host_monotonic_ns=receipt_ns, host_time=receipt_utc
            )
            if self.emg_classifier is not None and self.emg_classifier.error:
                self.error = self.emg_classifier.error
        finally:
            if writer is not None:
                with self._recording_idle:
                    try:
                        writer.handle(
                            sensor, payload, host_monotonic_ns=receipt_ns, host_time=receipt_utc
                        )
                    except (OSError, RuntimeError) as error:
                        self.recording_error = str(error)
                    finally:
                        self._recording_inflight -= 1
                        self._recording_idle.notify_all()

    def start_recording(self, session: Path) -> None:
        """Save packets from this connection without restarting calibration."""
        from aurora.recording.packet_writer import PacketWriter

        with self._recording_lock:
            if self._recording is not None or self._pending_writer is not None:
                raise RuntimeError("A BLE trial is already recording")
            self.recording_error = ""
            self._recording = PacketWriter(
                session,
                clock_sync=self.clock_sync,
                connection_generation=self.connection_generation,
            )
            self._recording_clock_epochs = [(self.connection_generation, self.clock_sync)]
            self._recording_device_resets = self.device_clock_resets

    def _start_connection(self):
        # A reconnect needs its own clock fit, including an active raw recording.
        with self._recording_lock:
            self.clock_sync = ClockSynchronizer()
            self.connection_generation += 1
            self.mounting_session_id = uuid.uuid4().hex
            self.decoder.reset_sequence_tracking()
            recorder = self._recording or (
                self._pending_writer if self._recording_inflight else None
            )
            if recorder is not None:
                recorder.clock_sync = self.clock_sync
                recorder.connection_generation = self.connection_generation
                self._recording_clock_epochs.append((self.connection_generation, self.clock_sync))

    def calibration_profile(self) -> dict:
        with self._lock:
            estimator = self.imu_estimator
            return {
                "format_version": 2,
                "side": estimator.side,
                "imu_motion_gap": self.imu_motion_gap,
                "imu_invalid_reason": self.imu_invalid_reason,
                "angle_units": "degrees",
                "angle_convention": "unclipped; wrist_relative_to_reference",
                "human_reference_channels": sorted(estimator.referenced_channels()),
                "body_reference": "camera_or_static_upright_assumption",
                "emg_baseline_adc": self.emg_estimator.baseline,
                "emg_rest": self.emg_estimator.rest_envelope,
                "emg_reference": self.emg_estimator.reference,
                "emg_strong": self.emg_estimator.strong_envelope,
                "emg_period": self._emg_period,
                "emg_model": self.emg_classifier.model.manifest if self.emg_classifier else None,
                "imu_tilt_offsets": dict(estimator.tilt_offsets),
                "imu_gyro_bias_z": dict(estimator.gyro_bias_z),
                "imu_elbow_zero": estimator.elbow_zero,
                "imu_camera_offsets": dict(estimator.camera_offsets),
                "imu_reference_angles": dict(estimator.reference_angles),
                "imu_filtered_accel": dict(estimator.filtered_accel),
                "imu_yaw_delta": dict(estimator.yaw_delta),
                "imu_shoulder_twist_deg": estimator.twist.shoulder_angle,
                "imu_wrist_twist_deg": estimator.twist.wrist_angle,
                "imu_twist_axes": dict(estimator.twist.axes),
                "imu_twist_bias_dps": dict(estimator.twist.bias),
                "connection_generation": self.connection_generation,
                "mounting_session_id": self.mounting_session_id,
                "sensor_alignment": self.alignment_profile,
            }

    def apply_alignment(self, profile, *, subject_id, side) -> None:
        """Accept a fit only for this subject, mounting and live connection."""
        with self._lock:
            if (
                profile.get("subject_id") != subject_id
                or profile.get("side") != side
                or profile.get("connection_generation") != self.connection_generation
                or profile.get("mounting_session_id") != self.mounting_session_id
            ):
                raise ValueError(
                    "Alignment belongs to another subject, arm or connection; repeat preparation"
                )
            self.imu_estimator.set_alignment(profile)
            self.alignment_profile = profile
            self.angles = None

    def begin_reference_capture(self, subject_id):
        from aurora.sensors.reference_capture import ReferenceCapture

        with self._lock:
            capture = ReferenceCapture(
                subject_id=subject_id,
                side=self.imu_estimator.side,
                connection_generation=self.connection_generation,
                mounting_session_id=self.mounting_session_id,
            )
            self.reference_capture = capture
            return capture

    def detach_reference_capture(self, capture):
        with self._lock:
            if self.reference_capture is capture:
                self.reference_capture = None

    def stop_recording(self, timeout=5.0) -> dict:
        deadline = time.monotonic() + timeout
        with self._recording_idle:
            recorder = self._recording or self._pending_writer
            if recorder is None:
                return {}
            # Handoff first; callbacks stay short while the writer drains.
            self._recording = None
            self._pending_writer = recorder
            while self._recording_inflight:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("Received BLE packets are still processing; retry saving")
                self._recording_idle.wait(remaining)
        recorder.close(timeout=max(0.0, deadline - time.monotonic()))
        with self._recording_lock:
            self._pending_writer = None
        return {
            **recorder.summary(),
            "time_sync": self.clock_sync.summary(),
            "time_sync_epochs": [
                {"connection_generation": generation, **synchronizer.summary()}
                for generation, synchronizer in self._recording_clock_epochs
            ],
            "write_error": self.recording_error or recorder.error,
            "device_clock_resets": self.device_clock_resets - self._recording_device_resets,
        }

    def flush_recording(self, timeout=5.0) -> None:
        """Wait for queued recording writes without holding the acquisition lock."""
        with self._recording_lock:
            writer = self._recording
        if writer is not None:
            writer.flush(timeout)

    def calibrate_emg_strength(self) -> bool:
        with self._lock:
            started = self.emg_estimator.calibrate_strength()
            if started:
                self.activation = None
            return started

    def graph_data(self) -> dict:
        """Copy recent received samples for the UI; stale streams show no trace."""
        now = self.clock()
        with self._lock:
            emg_fresh = self.emg_time > 0 and now - self.emg_time <= 0.5
            imu_fresh = self.imu_time > 0 and now - self.imu_time <= 0.5
            return {
                "emg": list(self._emg_history) if emg_fresh else [],
                "emg_period": self._emg_period,
                "activation": self.activation if emg_fresh else None,
                "emg_calibration": self.emg_estimator.status,
                "emg_calibration_state": self.emg_estimator.calibration_state,
                "imu_motion_gap": self.imu_motion_gap,
                "emg_envelope": self.emg_estimator.envelope if emg_fresh else None,
                "emg_rest": self.emg_estimator.rest_envelope,
                "emg_reference": self.emg_estimator.reference,
                "emg_strong": self.emg_estimator.strong_envelope,
                "emg_threshold": self.emg_estimator.activation_threshold,
                "emg_phase": self.emg_classifier.phase
                if self.emg_classifier and emg_fresh
                else None,
                "imus": {
                    role: list(samples)
                    if imu_fresh and now - self._imu_role_receipts.get(role, self.imu_time) <= 0.5
                    else []
                    for role, samples in self._imu_history.items()
                },
                "health": self.decoder.packet_health(),
                "status": self.status,
                "error": self.error,
                "emg_inference": {
                    "skipped_windows": self.emg_classifier.skipped_windows,
                    "error": self.emg_classifier.error,
                }
                if self.emg_classifier
                else None,
            }

    def _samples(self, sensor: str, _sequence: int, channels: list) -> None:
        if sensor not in ("imu", "emg"):
            return
        now = self.clock()
        with self._lock:
            packet = self.decoder.last_packet
            stamp = (
                packet.first_time_us
                if sensor == "emg"
                else next(
                    (
                        value
                        for values, value in zip(
                            packet.channels, packet.sample_times_us, strict=True
                        )
                        if values
                    ),
                    None,
                )
            )
            previous_time = (
                self._previous_emg_device_time if sensor == "emg" else self._previous_imu_time
            )
            last_receipt = self.emg_time if sensor == "emg" else self.imu_time
            if (
                stamp is not None
                and previous_time is not None
                and stamp < previous_time - 500_000
                and now - last_receipt > 0.5
            ):
                # V1 has no boot ID. A large backwards clock after stale input
                # starts an unsynchronized epoch; require reconnect/recalibration.
                self._start_connection()
                self._clear_state()
                self.device_clock_resets += 1
                self.error = "Possible device clock reset: reconnect and calibrate again"
            if sensor == "emg":
                if self._previous_emg_sequence is not None:
                    difference = (_sequence - self._previous_emg_sequence) & 0xFFFFFFFF
                    if difference == 0 or difference >= 0x80000000:
                        return  # Duplicate/reordered data is not a new muscle sample.
                    if difference > 1:
                        self.emg_estimator.sample_gap()
                        if self.emg_classifier is not None:
                            self.emg_classifier.reset()
                self._previous_emg_sequence = _sequence
                expected_time = self._previous_emg_end_us
                if packet.first_time_us is not None and packet.period_us is not None:
                    if expected_time is not None and abs(packet.first_time_us - expected_time) > 1:
                        self.emg_estimator.sample_gap()
                        if self.emg_classifier is not None:
                            self.emg_classifier.reset()
                        if packet.first_time_us < expected_time:
                            self.activation = None
                            self.error = "Overlapping EMG sample times; retaining raw packets only"
                            return
                    self._previous_emg_end_us = (
                        packet.first_time_us + len(channels[0]) * packet.period_us
                    )
                self._previous_emg_device_time = packet.first_time_us
                period = (packet.period_us or 10_000) / 1e6
                if period != self._emg_period:
                    self._emg_period = period
                    self.emg_estimator = EmgActivationEstimator(max(2, round(0.5 / period)))
                    self.activation = None
                    if self.emg_classifier is not None:
                        self.emg_classifier.reset()
                values = channels[0] if channels else []
                if not values:
                    self.emg_time = 0.0
                    self.activation = None
                    if self.emg_classifier is not None:
                        self.emg_classifier.reset()
                    return
                if (
                    self.emg_classifier is not None
                    and not abs(self._emg_period * self.emg_classifier.model.settings.rate_hz - 1)
                    < 0.01
                ):
                    self.emg_classifier.reset()
                    self.error = "EMG model sample rate differs from the BLE stream"
                    self.activation = None
                    self.emg_time = 0.0
                    return
                for value in values:
                    self.activation = self.emg_estimator.update(value, self._emg_period)
                    if self.emg_classifier is not None:
                        self.emg_classifier.update(value)
                self._emg_history.extend(values)
                self.emg_raw = int(values[-1])
                self.emg_time = now
            elif sensor == "imu":
                if self._previous_imu_sequence is not None:
                    difference = (_sequence - self._previous_imu_sequence) & 0xFFFFFFFF
                    if difference == 0 or difference >= 0x80000000:
                        return
                self._previous_imu_sequence = _sequence
                self._imu_device_times = {
                    role: stamp
                    for role, stamp, samples in zip(
                        ("shoulder", "wrist"), packet.sample_times_us, packet.channels, strict=False
                    )
                    if samples and stamp is not None
                }
                self._imu_device_time = next(iter(self._imu_device_times.values()), None)
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
                        "sample_time_us": self._imu_device_times.get(role),
                        "host_monotonic_ns": round(now * 1e9),
                        "connection_generation": self.connection_generation,
                        "gyro_saturated": (0 if role == "shoulder" else 1)
                        in packet.gyro_saturated_roles,
                        "validity": "gyro_at_wire_limit"
                        if (0 if role == "shoulder" else 1) in packet.gyro_saturated_roles
                        else "valid",
                        "accel_g": dict(zip("xyz", (v * scale for v in values[:3]), strict=True)),
                        "gyro_dps": dict(zip("xyz", (v * scale for v in values[3:6]), strict=True)),
                    }
                dt = now - self.imu_time if self.imu_time else 0.01
                if self._imu_device_time is not None and self._previous_imu_time is not None:
                    dt = (self._imu_device_time - self._previous_imu_time) / 1_000_000
                if dt <= 0 or dt > 0.5:
                    dt = 0.01
                self._previous_imu_time = self._imu_device_time
                role_dt = {}
                updated_roles = set()
                for role, stamp in self._imu_device_times.items():
                    previous = self._previous_imu_times.get(role)
                    if previous is None:
                        role_dt[role] = 0.01
                    elif stamp > previous:
                        elapsed = (stamp - previous) / 1e6
                        role_dt[role] = elapsed if elapsed <= 0.5 else 0.0
                        if elapsed > 0.5:
                            self.imu_motion_gap = True
                            self.imu_invalid_reason = (
                                "IMU timing gap: motion during the gap is unknown"
                            )
                    else:
                        role_dt[role] = 0.0
                    if previous is None or stamp > previous:
                        self._previous_imu_times[role] = stamp
                        self._imu_role_receipts[role] = now
                        updated_roles.add(role)
                    elif now - self._imu_role_receipts.get(role, 0.0) > 0.5:
                        sensors.pop(role, None)
                    if role in sensors and sensors[role]["gyro_saturated"]:
                        role_dt[role] = 0.0
                        self.imu_motion_gap = True
                        self.imu_invalid_reason = (
                            "Gyroscope at the BLE wire limit: rotation rate is unknown"
                        )
                for role, sensor in sensors.items():
                    sensor["host_monotonic_ns"] = round(
                        self._imu_role_receipts.get(role, now) * 1e9
                    )
                self.sensors = sensors
                if self.reference_capture is not None:
                    self.reference_capture.add_imus(
                        {
                            role: {
                                **sensor,
                                "host_monotonic_ns": self.decoder.last_host_monotonic_ns,
                            }
                            for role, sensor in sensors.items()
                            if role in updated_roles
                        }
                    )
                for role, history in self._imu_history.items():
                    if role not in sensors:
                        history.clear()
                        continue
                    if role_dt and role not in updated_roles:
                        continue
                    sensor = sensors[role]
                    history.append(
                        tuple(sensor["accel_g"][axis] for axis in "xyz")
                        + tuple(sensor["gyro_dps"][axis] for axis in "xyz")
                    )
                if role_dt and not any(role_dt.values()):
                    if updated_roles:
                        self.imu_time = now if sensors else 0.0
                    return
                self.angles = self.imu_estimator.update(sensors, dt, sample_dt_s=role_dt or None)
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
                target = self.address or await find_device(BleakScanner, self.device, timeout=10.0)
                if target is None:
                    raise RuntimeError(f"{self.device} not found")
                async with BleakClient(target) as client:
                    self._start_connection()
                    sync_client = None
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
                    try:
                        if self.synchronize_clock:
                            sync_client = BleClockSyncClient(client, self.clock_sync)
                            await sync_client.start()
                        while client.is_connected and not self._stop.is_set():
                            await asyncio.sleep(0.1)
                    finally:
                        if sync_client is not None:
                            await sync_client.close()
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
