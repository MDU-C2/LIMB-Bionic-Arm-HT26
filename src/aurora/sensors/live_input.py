# Majd, Christian
"""Reusable OAK-D and one/two-IMU input for the interactive AURORA simulator."""

from __future__ import annotations

import contextlib
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.camera.hand_tracking import ArmHandTracker
from aurora.sensors.camera.oak_camera import (
    FRAME_SIZE,
    CameraFrame,
    PreviewFrames,
    build_pipeline,
    configure_depthai_runtime,
    receive_frame,
    require_pose_model,
)
from aurora.sensors.camera.oak_tracking import analyze_frame
from aurora.sensors.emg_protocol import EmgActivationEstimator, extract_emg, select_grip_source
from aurora.sensors.fusion import (
    ControlAngleSmoother,
    DualImuArmEstimator,
    camera_control_angles,
    camera_hand_curl,
    fuse_control_angles,
    interactive_control_targets,
    retarget_reference,
)
from aurora.sensors.imu_protocol import extract_imus, imu_configuration
from aurora.sensors.imu_twist import DEFAULT_TWIST_AXIS
from aurora.sensors.serial_input import SerialPacketReader
from aurora.sensors.settings import DEFAULT_BLE_DEVICE_NAME, DEFAULT_GRIP_SOURCE

WINDOW = "AURORA | Live camera monitor"
CameraFrameSnapshot = CameraFrame


def installed_pose_model_complexity(mp) -> int:
    """Use the heavy pose model if installed, otherwise use the bundled full model."""
    package_directory = Path(mp.__file__).resolve().parent
    heavy_model = package_directory / "modules" / "pose_landmark" / "pose_landmark_heavy.tflite"
    return 2 if heavy_model.is_file() else 1


@dataclass
class LiveFusionSnapshot:
    """Latest control and monitoring values from both live sources."""

    angles: dict[str, float] | None
    sensors: dict[str, dict[str, Any]]
    camera_ready: bool
    imu_ready: bool
    status: str
    hand_curl: float | None = None
    grip_source: str = "--"
    emg_raw: int | None = None
    emg_activation: float | None = None
    stop_requested: bool = False
    rotation_source: str = "--"
    robot_targets: dict[str, float] | None = None
    body_reference: str = "unavailable"
    return_error_deg: float | None = None
    calibration_valid: bool = True
    relative_angles: dict[str, float] | None = None


def _draw_overlay(
    frame,
    status: str,
    fused: dict[str, float] | None,
    cv2,
    hand_curl: float | None = None,
    grip_source: str = "--",
):
    """Render compact tracking and control telemetry over the camera image."""
    canvas = frame.copy()
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 92), (18, 25, 39), -1)
    cv2.putText(
        canvas,
        status[:90],
        (14, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (230, 237, 248),
        1,
        cv2.LINE_AA,
    )
    if fused:

        def angle(name: str) -> str:
            return f"{fused[name]:.1f}" if name in fused else "--"

        grip = f"{hand_curl * 100:.0f}% ({grip_source})" if hand_curl is not None else "--"
        values = (
            f"Flex {angle('shoulder_flexion')}  "
            f"L/R {angle('shoulder_abduction')}  "
            f"axial {angle('shoulder_rotation_proxy')}  "
            f"elbow {angle('elbow_flexion')}  grip {grip}"
        )
    else:
        values = "Waiting for a camera pose or an IMU"
    cv2.putText(
        canvas,
        values,
        (14, 54),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (85, 220, 170),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "Complete setup in the sensor window   Q: stop control",
        (14, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (174, 187, 207),
        1,
        cv2.LINE_AA,
    )
    return canvas


class LiveSensorInput:
    """Own the live camera window and an ESP32 BLE or serial connection."""

    def __init__(
        self,
        port: str | None = None,
        baud: int = 115200,
        side: str = "left",
        camera_weight: float = 0.25,
        depth: bool = False,
        transport: str | None = None,
        device: str = DEFAULT_BLE_DEVICE_NAME,
        address: str = "",
        grip_source: str = DEFAULT_GRIP_SOURCE,
        shoulder_twist_axis: str = DEFAULT_TWIST_AXIS,
        wrist_twist_axis: str = DEFAULT_TWIST_AXIS,
        synchronize_clock: bool = False,
        show_monitor: bool = True,
        pose_model_complexity: int | None = None,
        emg_model: Path | None = None,
        clock=None,
    ) -> None:
        self.port = port
        self.clock = clock or (lambda: time.monotonic())
        self.baud = baud
        self.side = side
        self.camera_weight = camera_weight
        self.depth = depth
        self.transport = transport or ("serial" if port else "ble")
        self.device = device
        self.address = address
        if grip_source not in ("auto", "camera", "emg"):
            raise ValueError("Grip source must be auto, camera, or emg")
        self.grip_source = grip_source
        if emg_model is not None and (self.transport != "ble" or grip_source != "emg"):
            raise ValueError("An EMG model requires BLE transport and the EMG grip source.")
        self.emg_model = emg_model
        self.model_grip = None
        self.emg_phase = None
        self.shoulder_twist_axis = shoulder_twist_axis
        self.wrist_twist_axis = wrist_twist_axis
        self.synchronize_clock = synchronize_clock
        self.show_monitor = show_monitor
        self.pose_model_complexity = pose_model_complexity
        self.reader = None
        self.pipeline = None
        self.pose = None
        self.hands = None
        self.cup_detector = None
        self.depth_queue = None
        self.video_queue = None
        self.intrinsics = None
        self._camera_frames = PreviewFrames(clock=self.clock)
        self.latest_frame = None
        self.camera_sample = None
        self.camera_observation = {}
        self.camera_body_reference = False
        self.camera_frame = None
        self.frame_sequence = 0
        self.frame_time_ns = 0
        self.camera_status = "Waiting for OAK-D frame"
        self.camera_angles: dict[str, float] = {}
        self.raw_camera_angles: dict[str, float] = {}
        self.reference_angles: dict[str, float] | None = None
        self.robot_reference_angles: dict[str, float] = {}
        self.camera_origin: dict[str, float] = {}
        self.camera_time = 0.0
        self.hand_curl: float | None = None
        self.hand_time = 0.0
        self.emg_raw: int | None = None
        self.emg_activation: float | None = None
        self.emg_time = 0.0
        self.previous_camera_time = self.clock()
        self.imu_angles: dict[str, float] | None = None
        self.imu_time = 0.0
        self.imu_packet_time = 0.0
        self.previous_imu_time = self.clock()
        self._serial_sample_times = {}
        self._serial_emg_time = None
        self._serial_epoch = None
        self.sensors: dict[str, dict[str, Any]] = {}
        self.imu_mode = "none"
        self.imu_roles: tuple[str, ...] = ()
        self.estimator = DualImuArmEstimator(
            shoulder_twist_axis=shoulder_twist_axis, wrist_twist_axis=wrist_twist_axis, side=side
        )
        self.emg_estimator = EmgActivationEstimator()
        self.camera_smoother = ControlAngleSmoother()
        self._closed = False
        self._source_lock = threading.RLock()
        self._worker_stop = threading.Event()
        self._worker = None
        self._pending_camera = None
        self._worker_error = ""
        self._connection_generation = None
        self._calibration_valid = False
        self._motion_gap = False
        self._imu_invalid_reason = ""

    def start(self) -> None:
        """Start hardware and show the camera monitor."""
        if self._closed or self.reader is not None or self.pipeline is not None:
            raise RuntimeError("Input already started or closed; release it and create a new input")
        configure_depthai_runtime()
        try:
            import cv2
            import depthai as dai
            import mediapipe as mp
            import numpy as np
        except ImportError as error:
            raise RuntimeError(f"Live control dependency missing: {error.name}") from error

        self.cv2 = cv2
        cv2.setNumThreads(1)
        self.mp = mp
        self.np = np
        if self.transport == "ble":
            self.reader = BleSensorReader(
                self.device,
                self.address,
                self.shoulder_twist_axis,
                self.wrist_twist_axis,
                synchronize_clock=self.synchronize_clock,
                emg_model=self.emg_model,
                clock=self.clock,
                side=self.side,
            )
        else:
            import serial

            self.reader = SerialPacketReader(serial, self.port, self.baud)
        try:
            self.reader.start()
            self.pipeline, self.video_queue, self.depth_queue = build_pipeline(dai, self.depth)
            self.pipeline.start()
            if self.depth:
                calibration = self.pipeline.getDefaultDevice().readCalibration()
                self.intrinsics = calibration.getCameraIntrinsics(
                    dai.CameraBoardSocket.CAM_A, *FRAME_SIZE
                )
            pose_complexity = self.pose_model_complexity
            if pose_complexity is None:
                pose_complexity = installed_pose_model_complexity(mp)
            require_pose_model(mp, pose_complexity)
            self.pose = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=pose_complexity,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            model_name = "heavy" if pose_complexity == 2 else "full"
            self.camera_status = f"Waiting for OAK-D frame ({model_name} pose model)"
            if self.grip_source != "emg":
                self.hands = ArmHandTracker(mp, cv2)
            if self.show_monitor:
                cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
                cv2.moveWindow(WINDOW, 540, 420)
        except Exception:
            self.close()
            raise

    def invalidate_calibration(self) -> None:
        with self._source_lock:
            self._calibration_valid = False
            self.estimator.clear_alignment()
            self.emg_estimator.reset()
            self.emg_activation = None
            if self.transport == "ble" and self.reader is not None:
                self.reader.invalidate_calibration()

    def calibrate(self, reference_angles: object = None) -> None:
        with self._source_lock:
            self._calibrate(reference_angles)

    def _calibrate(self, reference_angles: object = None) -> None:
        self._calibration_valid = True
        self._motion_gap = False
        self._imu_invalid_reason = ""
        self.robot_reference_angles = camera_control_angles(reference_angles)
        self.camera_origin = (
            dict(self.raw_camera_angles) if self.clock() - self.camera_time <= 0.5 else {}
        )
        self.reference_angles = dict(self.camera_origin)
        self.estimator.reset_calibration(self.reference_angles)
        if self.transport == "ble" and self.reader is not None:
            self.reader.calibrate(reset_emg=False, reference_angles=self.reference_angles)
        self.camera_smoother.reset()
        self.imu_angles = None
        self.imu_time = 0.0
        self.camera_angles = {}
        self.camera_time = 0.0

    def calibrate_emg(self) -> None:
        with self._source_lock:
            self.emg_estimator.reset()
            self.emg_activation = None
            if self.transport == "ble" and self.reader is not None:
                self.reader.calibrate_emg()

    def calibrate_emg_strength(self) -> bool:
        if self.transport == "ble" and self.reader is not None:
            return self.reader.calibrate_emg_strength()
        return self.emg_estimator.calibrate_strength()

    def start_background(self) -> None:
        """Keep camera inference out of the simulation and GUI loops."""
        if self._closed or self._worker_stop.is_set():
            raise RuntimeError("Input is closing or closed")
        if self._worker is not None:
            return
        self._worker_stop.clear()
        self._worker = threading.Thread(target=self._acquire, daemon=True)
        self._worker.start()

    def _acquire(self) -> None:
        while not self._worker_stop.is_set():
            try:
                self._poll_camera()
            except Exception as error:
                self._worker_error = f"Input stopped: {error}"
                return
            self._worker_stop.wait(0.003)

    def poll(self) -> LiveFusionSnapshot:
        if self._worker_stop.is_set():
            return LiveFusionSnapshot(None, {}, False, False, "Inputs stopped", stop_requested=True)
        # The control caller owns fusion state. The worker only publishes camera
        # results; neither inference nor serial queue consumption runs twice.
        snapshot = self._poll_sources()
        if self._worker_error:
            snapshot = replace(snapshot, status=self._worker_error, stop_requested=True)
        if self.show_monitor:
            return self._render_monitor(snapshot)
        return snapshot

    def _poll_sources(self) -> LiveFusionSnapshot:
        """Consume one published observation and advance control without inference."""
        if self._worker is None:
            self._poll_camera()
        with self._source_lock:
            pending, self._pending_camera = self._pending_camera, None
            if pending is not None:
                camera, status, received_at = pending
                self.camera_frame = camera
                self.latest_frame = camera.latest_frame
                self.frame_sequence = camera.frame_sequence
                self.frame_time_ns = camera.frame_time_ns
                self.camera_sample = camera.camera_sample
                self.camera_observation = camera.camera_observation
                self.camera_status = status
                self.accept_camera_observation(camera.camera_observation, received_at)
            return self._snapshot()

    def _poll_camera(self) -> None:
        """Run camera work outside the control lock and publish one complete frame."""
        camera, depth = self._camera_frames.poll(
            self.video_queue, self.depth_queue, receive=receive_frame
        )
        if camera is None:
            return
        matched_depth = depth.depth_mm if depth is not None else None
        image, status, sample, observation = analyze_frame(
            camera.latest_frame,
            matched_depth,
            self.pose,
            self.mp,
            self.cv2,
            self.np,
            self.side,
            self.intrinsics,
            self.hands,
            cup_detector=self.cup_detector,
        )
        if depth is not None:
            observation.update(
                depth_device_time_ns=depth.device_time_ns,
                depth_time_difference_ns=depth.device_time_ns - camera.device_time_ns,
            )
        camera = replace(
            camera, latest_frame=image, camera_sample=sample, camera_observation=observation
        )
        with self._source_lock:
            self._pending_camera = camera, status, camera.received_at_s

    def _read_packet(self, now: float) -> None:
        if self.transport == "serial" and isinstance(self.reader, SerialPacketReader):
            for packet in self.reader.drain():
                self._accept_serial_packet(packet, now)
            return
        packet = self.reader.latest() if self.reader is not None else None
        if packet is not None and self.transport == "ble":
            generation = packet.get("connection_generation")
            if (
                self._connection_generation is not None
                and generation != self._connection_generation
            ):
                self._calibration_valid = False
                self._motion_gap, self._imu_invalid_reason = False, ""
                self.reference_angles = None
                self.robot_reference_angles = {}
                self.camera_origin = {}
            self._connection_generation = generation
            if packet.get("imu_motion_gap"):
                self._motion_gap = True
                self._imu_invalid_reason = packet.get("imu_invalid_reason", "")
                self._calibration_valid = False
            self.sensors = extract_imus(packet)
            self.imu_mode, self.imu_roles = imu_configuration(packet)
            self.imu_angles = packet["imu_angles"]
            self.imu_packet_time = packet["imu_time"]
            self.imu_time = packet["imu_time"]
            self.emg_raw = packet["emg_raw"]
            self.emg_activation = packet["emg_activation"]
            self.emg_time = packet["emg_time"]
            self.model_grip = packet.get("emg_model_grip")
            self.emg_phase = packet.get("emg_phase")
        elif packet is not None:
            self._accept_serial_packet(packet, now)

    def feed_serial(self, packet) -> None:
        """Feed an externally owned serial connection into the headless processor."""
        if self._closed or self.transport != "serial" or self.reader is not None:
            raise RuntimeError("External serial input needs a processor without its own connection")
        with self._source_lock:
            self._accept_serial_packet(packet, self.clock())

    def _accept_serial_packet(self, packet, now: float) -> None:
        epoch = packet.get("connection_generation")
        if self._serial_epoch is not None and epoch != self._serial_epoch:
            self._serial_sample_times.clear()
            self._serial_emg_time = None
            self._calibration_valid = False
            self._motion_gap, self._imu_invalid_reason = False, ""
            self.reference_angles = None
            self.robot_reference_angles, self.camera_origin = {}, {}
            self.estimator.clear_alignment()
            self.emg_estimator.reset()
            self.imu_angles = None
            self.imu_time = self.imu_packet_time = self.emg_time = 0.0
            self.emg_raw = self.emg_activation = self.model_grip = None
        self._serial_epoch = epoch
        sensors = extract_imus(packet)
        received = packet.get("host_monotonic_ns", round(now * 1e9)) / 1e9
        sample_dt = {}
        for role, sensor in sensors.items():
            sensor["connection_generation"] = epoch
            if sensor.get("gyro_saturated"):
                self._calibration_valid = False
                self._motion_gap = True
                self._imu_invalid_reason = "Gyroscope at the wire limit: rotation rate is unknown"
            host_time = sensor.get("host_monotonic_ns", round(received * 1e9)) / 1e9
            if now - host_time > 0.5:
                sensor["connected"] = False
            stamp = sensor.get("sample_time_us", round(host_time * 1e6))
            previous = self._serial_sample_times.get(role)
            if sensor.get("connected") and (previous is None or stamp > previous):
                elapsed = (stamp - previous) / 1e6 if previous is not None else 0.01
                if elapsed < 0 or elapsed > 0.5:
                    self._calibration_valid = False
                    self._motion_gap = True
                    elapsed = 0.0
                sample_dt[role] = elapsed
                self._serial_sample_times[role] = stamp
        self.sensors = sensors
        self.imu_mode, self.imu_roles = imu_configuration({"imus": sensors})
        if sample_dt:
            self.imu_packet_time = received
            self.imu_angles = self.estimator.update(sensors, 0.01, sample_dt_s=sample_dt)
            self.imu_time = received if self.imu_angles else 0.0
        emg = extract_emg(packet)
        updated = packet.get("updated_sensor")
        if emg.get("connected") and updated in (None, "emg"):
            stamp = packet.get("sample_time_us", round(received * 1e6))
            if stamp != self._serial_emg_time:
                elapsed = (stamp - self._serial_emg_time) / 1e6 if self._serial_emg_time else 0.01
                self._serial_emg_time = stamp
                self.emg_raw = int(emg["adc_raw"])
                self.emg_activation = self.emg_estimator.update(self.emg_raw, elapsed)
                self.emg_time = received

    def _snapshot(self) -> LiveFusionSnapshot:
        now = self.clock()
        self._read_packet(now)
        if self.transport == "serial":
            from aurora.sensors.fusion import fresh_imu_angles

            for sensor in self.sensors.values():
                if now - sensor.get("host_monotonic_ns", 0) / 1e9 > 0.5:
                    sensor["connected"] = False
            self.imu_mode, self.imu_roles = imu_configuration({"imus": self.sensors})
            self.imu_angles = fresh_imu_angles(self.imu_angles, self.imu_roles)
        recent_imu = (
            self.imu_angles if not self._motion_gap and now - self.imu_time <= 0.5 else None
        )
        recent_camera = self.camera_angles if now - self.camera_time <= 0.5 else None
        recent_hand_curl = self.hand_curl if now - self.hand_time <= 0.5 else None
        recent_emg = self.emg_activation if now - self.emg_time <= 0.5 else None
        control_emg = (
            self.model_grip if self.emg_model is not None and recent_emg is not None else recent_emg
        )
        grip, grip_source = select_grip_source(control_emg, recent_hand_curl, self.grip_source)
        if self.emg_model is not None:
            grip_source = f"EMG model: {self.emg_phase or 'warming up'}"
        imu_connected = self.imu_mode != "none" and now - self.imu_packet_time <= 0.5
        fused = fuse_control_angles(
            recent_imu, recent_camera, self.camera_weight, camera_corrected=True
        )
        human_names = set(self.reference_angles or {})
        estimator = (
            self.reader.imu_estimator
            if isinstance(self.reader, BleSensorReader)
            else self.estimator
        )
        human_names.update(estimator.referenced_channels())
        human_names.update(
            name
            for name in recent_camera or {}
            if name not in (recent_imu or {}) or self.camera_weight == 1
        )
        anatomical = {name: value for name, value in (fused or {}).items() if name in human_names}
        relative = {name: value for name, value in (fused or {}).items() if name not in human_names}
        imu_rotation = (recent_imu or {}).get("shoulder_rotation_proxy")
        camera_rotation = (recent_camera or {}).get("shoulder_rotation_proxy")
        if imu_rotation is not None:
            rotation_source = (
                "IMU + camera" if camera_rotation is not None and self.camera_weight > 0 else "IMU"
            )
        else:
            rotation_source = "camera" if camera_rotation is not None else "--"
        if imu_connected and self.imu_mode == "dual":
            imu_status = "IMUs 2/2"
        elif imu_connected and self.imu_mode == "single":
            imu_status = f"IMU 1/2 ({self.imu_roles[0]})"
        else:
            imu_status = "IMUs --"
        emg_status = (
            f"EMG {recent_emg * 100:.0f}%"
            if recent_emg is not None
            else "EMG calibrating"
            if now - self.emg_time <= 0.5
            else "EMG --"
        )
        source_status = (
            f"Camera {'OK' if recent_camera else '--'} | {imu_status} | {emg_status} | "
            f"Grip {grip_source if grip is not None else 'waiting for ' + self.grip_source}"
        )
        body_reference = (
            "camera_trunk"
            if recent_camera and self.camera_body_reference
            else "stationary_trunk_assumed"
            if recent_imu
            else "unavailable"
        )
        if body_reference == "stationary_trunk_assumed":
            source_status += " | Body reference unavailable: shoulder assumes a still trunk"
        if relative:
            source_status += " | Unreferenced IMU channels show change since zeroing"
        if self._motion_gap:
            source_status += (
                " | "
                + (self._imu_invalid_reason or "IMU timing gap: motion during the gap is unknown")
                + "; repeat alignment"
            )
        return_errors = [
            abs(self.raw_camera_angles[name] - origin)
            for name, origin in self.camera_origin.items()
            if recent_camera
            and name in self.raw_camera_angles
            and name != "shoulder_rotation_proxy"
        ]
        if self.reader is not None and self.reader.error:
            source_status += f" | {self.transport.upper()}: {self.reader.error}"
        elif self.transport == "ble" and self.reader is not None:
            source_status += f" | BLE {self.reader.status}"

        stop_requested = False
        if self.pipeline is not None and not self.pipeline.isRunning():
            source_status = "OAK-D pipeline stopped"
            stop_requested = True

        return LiveFusionSnapshot(
            angles=anatomical or None,
            sensors=self.sensors,
            camera_ready=recent_camera is not None,
            imu_ready=imu_connected,
            status=f"{source_status} | {self.camera_status}",
            hand_curl=grip,
            grip_source=grip_source,
            emg_raw=self.emg_raw if now - self.emg_time <= 0.5 else None,
            emg_activation=recent_emg,
            stop_requested=stop_requested,
            rotation_source=rotation_source,
            robot_targets=interactive_control_targets(
                retarget_reference(
                    fused,
                    self.reference_angles
                    or (
                        dict.fromkeys(self.robot_reference_angles, 0.0)
                        if self._calibration_valid
                        else None
                    ),
                    self.robot_reference_angles,
                )
            ),
            body_reference=body_reference,
            return_error_deg=max(return_errors) if recent_camera and return_errors else None,
            calibration_valid=self._calibration_valid,
            relative_angles=relative or None,
        )

    def accept_camera_observation(self, observation, now: float) -> None:
        """Use the same alignment and camera correction for live and replay data."""
        if self.clock() - now > 0.5:
            self.camera_body_reference = False
            self.camera_status = "Camera result too old for control"
            return
        raw_camera_angles = camera_control_angles(
            observation.get("angles_deg") if isinstance(observation, dict) else None
        )
        self.raw_camera_angles = dict(raw_camera_angles)
        self.camera_body_reference = bool(
            "shoulder_flexion" in raw_camera_angles and observation.get("reference_visible", True)
        )
        camera_dt = now - self.previous_camera_time
        self.previous_camera_time = now
        measured = self.camera_smoother.update(
            raw_camera_angles,
            camera_dt,
        )
        if measured:
            self.camera_angles = measured
            self.camera_time = now
            if self.camera_weight > 0:
                estimator = (
                    self.reader.imu_estimator
                    if isinstance(self.reader, BleSensorReader)
                    else self.estimator
                )
                unreferenced = {
                    name: value
                    for name, value in (self.imu_angles or {}).items()
                    if name not in estimator.referenced_channels()
                }
                # Correct once per new observation, not once per rendered loop.
                correction_weight = 1.0 - (1.0 - self.camera_weight) ** (
                    min(1.0, max(0.001, camera_dt)) / (1.0 / 30.0)
                )
                corrected = None
                if self.transport == "ble" and self.reader is not None:
                    corrected = self.reader.correct_angles(measured, correction_weight)
                    if corrected is not None:
                        self.imu_angles = corrected
                elif now - self.imu_time <= 0.5 and self.imu_angles is not None:
                    corrected = self.estimator.correct_angles(measured, correction_weight)
                    self.imu_angles = corrected
                # A newly observed anatomical origin must not jump a diagnostic
                # command that was already calibrated in relative coordinates.
                if self._calibration_valid:
                    for name, previous in unreferenced.items():
                        if name in measured and name in self.robot_reference_angles:
                            if corrected is None or name not in corrected:
                                self._calibration_valid = False
                            else:
                                if self.reference_angles is None:
                                    self.reference_angles = {}
                                self.reference_angles[name] = corrected[name] - previous
        measured_curl = camera_hand_curl(
            observation.get("finger_curl_deg") if isinstance(observation, dict) else None
        )
        if measured_curl is not None:
            self.hand_curl = measured_curl
            self.hand_time = now

    def _render_monitor(self, snapshot: LiveFusionSnapshot) -> LiveFusionSnapshot:
        if self.latest_frame is not None:
            canvas = _draw_overlay(
                self.latest_frame,
                snapshot.status,
                {**(snapshot.relative_angles or {}), **(snapshot.angles or {})},
                self.cv2,
                snapshot.hand_curl,
                snapshot.grip_source,
            )
            self.cv2.imshow(WINDOW, canvas)
        key = self.cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            snapshot = replace(snapshot, stop_requested=True)
        if (
            self.show_monitor
            and self.latest_frame is not None
            and hasattr(self.cv2, "getWindowProperty")
            and self.cv2.getWindowProperty(WINDOW, self.cv2.WND_PROP_VISIBLE) < 1
        ):
            snapshot = replace(snapshot, stop_requested=True)

        return snapshot

    def close(self) -> None:
        if self._closed:
            return
        self._worker_stop.set()
        errors = []
        if self._worker is not None:
            self._worker.join(timeout=5)
            if self._worker.is_alive():
                errors.append("Camera worker did not stop; camera resources are still in use")
            else:
                self._worker = None
        # The camera worker never uses the BLE/serial reader. Its cleanup is
        # independent even if inference is stuck. Camera resources need a join.
        resources = [("reader", "stop")]
        if self._worker is None:
            resources += [
                ("pose", "close"),
                ("hands", "close"),
                ("cup_detector", "close"),
                ("pipeline", "stop"),
            ]
        for name, method in resources:
            resource = getattr(self, name)
            if resource is None:
                continue
            try:
                if getattr(resource, method)() is False:
                    raise RuntimeError("worker did not stop; connection remains owned")
            except Exception as error:
                errors.append(f"{name}: {error}")
            else:
                setattr(self, name, None)
        if self.show_monitor and hasattr(self, "cv2"):
            with contextlib.suppress(self.cv2.error):
                self.cv2.destroyWindow(WINDOW)
        self._closed = not errors
        if errors:
            raise RuntimeError("; ".join(errors))
