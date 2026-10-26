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
from aurora.sensors.camera.cup_detection import CupDetector
from aurora.sensors.camera.hand_tracking import ArmHandTracker
from aurora.sensors.camera.oak_camera import (
    FRAME_SIZE,
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
)
from aurora.sensors.imu_protocol import extract_imus, imu_configuration
from aurora.sensors.imu_twist import DEFAULT_TWIST_AXIS
from aurora.sensors.serial_input import SerialPacketReader
from aurora.sensors.settings import DEFAULT_BLE_DEVICE_NAME, DEFAULT_GRIP_SOURCE

WINDOW = "AURORA | Live camera monitor"


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


@dataclass(frozen=True)
class CameraFrameSnapshot:
    frame_sequence: int
    frame_time_ns: int
    latest_frame: Any
    camera_sample: Any
    camera_observation: dict


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
        self.latest_depth = None
        self.latest_frame = None
        self.camera_sample = None
        self.camera_observation = {}
        self.camera_frame = None
        self.frame_sequence = 0
        self.frame_time_ns = 0
        self.camera_status = "Waiting for OAK-D frame"
        self.camera_angles: dict[str, float] = {}
        self.raw_camera_angles: dict[str, float] = {}
        self.reference_angles: dict[str, float] | None = None
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
        self.sensors: dict[str, dict[str, Any]] = {}
        self.imu_mode = "none"
        self.imu_roles: tuple[str, ...] = ()
        self.estimator = DualImuArmEstimator(
            shoulder_twist_axis=shoulder_twist_axis, wrist_twist_axis=wrist_twist_axis
        )
        self.emg_estimator = EmgActivationEstimator()
        self.camera_smoother = ControlAngleSmoother()
        self._closed = False
        self._source_lock = threading.RLock()
        self._worker_stop = threading.Event()
        self._worker = None
        self._background_snapshot = LiveFusionSnapshot(None, {}, False, False, "Starting inputs")

    def start(self) -> None:
        """Start hardware and show the camera monitor."""
        configure_depthai_runtime()
        try:
            import cv2
            import depthai as dai
            import mediapipe as mp
            import numpy as np
        except ImportError as error:
            raise RuntimeError(f"Live control dependency missing: {error.name}") from error

        self.cv2 = cv2
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
            )
        else:
            import serial

            self.reader = SerialPacketReader(serial, self.port, self.baud)
        self.reader.start()
        try:
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
            self.hands = ArmHandTracker(mp, cv2)
            self.cup_detector = CupDetector(cv2, np)
            if self.show_monitor:
                cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
                cv2.moveWindow(WINDOW, 540, 420)
        except Exception:
            self.close()
            raise

    def calibrate(self, reference_angles: object = None) -> None:
        with self._source_lock:
            self._calibrate(reference_angles)

    def _calibrate(self, reference_angles: object = None) -> None:
        if reference_angles is not None:
            self.reference_angles = camera_control_angles(reference_angles)
        self.camera_origin = (
            dict(self.raw_camera_angles) if self.clock() - self.camera_time <= 0.5 else {}
        )
        self.estimator.reset_calibration(self.reference_angles)
        if self.transport == "ble" and self.reader is not None:
            self.reader.calibrate(reset_emg=False, reference_angles=self.reference_angles)
        self.camera_smoother.reset()
        self.imu_angles = None
        self.imu_time = 0.0
        self.camera_angles = {}
        self.camera_time = 0.0

    def calibrate_emg(self) -> None:
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
        if self._worker is not None:
            return
        self._worker_stop.clear()
        self._worker = threading.Thread(target=self._acquire, daemon=True)
        self._worker.start()

    def _acquire(self) -> None:
        while not self._worker_stop.is_set():
            try:
                with self._source_lock:
                    self._background_snapshot = self._poll_sources()
            except Exception as error:
                self._background_snapshot = LiveFusionSnapshot(
                    None, {}, False, False, f"Input stopped: {error}", stop_requested=True
                )
                return
            self._worker_stop.wait(0.003)

    def poll(self) -> LiveFusionSnapshot:
        snapshot = (
            self._poll_sources()
            if self._worker is None
            else (
                self._background_snapshot
                if self._background_snapshot.stop_requested
                else self._snapshot()
            )
        )
        if self.show_monitor:
            return self._render_monitor(snapshot)
        return snapshot

    def _poll_sources(self) -> LiveFusionSnapshot:
        """Advance both sources once and refresh the camera monitor."""
        now = self.clock()

        if self.depth_queue is not None:
            depth_message = self.depth_queue.tryGet()
            if depth_message is not None:
                self.latest_depth = depth_message.getFrame()
        frame_message = self.video_queue.tryGet() if self.video_queue is not None else None
        if frame_message is not None:
            frame, frame_time_ns = receive_frame(frame_message)
            if frame is not None:
                self.frame_time_ns = frame_time_ns
                self.latest_frame, self.camera_status, self.camera_sample, observation = (
                    analyze_frame(
                        frame,
                        self.latest_depth,
                        self.pose,
                        self.mp,
                        self.cv2,
                        self.np,
                        self.side,
                        self.intrinsics,
                        self.hands,
                        cup_detector=self.cup_detector,
                    )
                )
                self.camera_observation = observation
                self.frame_sequence += 1
                self.accept_camera_observation(observation, now)
                self.camera_frame = CameraFrameSnapshot(
                    self.frame_sequence,
                    frame_time_ns,
                    self.latest_frame,
                    self.camera_sample,
                    observation,
                )

        return self._snapshot()

    def _read_packet(self, now: float) -> None:
        packet = self.reader.latest() if self.reader is not None else None
        if packet is not None and self.transport == "ble":
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
            self.sensors = extract_imus(packet)
            self.imu_mode, self.imu_roles = imu_configuration(packet)
            if self.imu_mode != "none":
                self.imu_packet_time = now
            dt = now - self.previous_imu_time
            self.previous_imu_time = now
            self.imu_angles = self.estimator.update(self.sensors, dt)
            if self.imu_angles:
                self.imu_time = now
            emg = extract_emg(packet)
            if emg.get("connected") is True:
                self.emg_raw = int(emg["adc_raw"])
                self.emg_activation = self.emg_estimator.update(self.emg_raw, dt)
                self.emg_time = now

    def _snapshot(self) -> LiveFusionSnapshot:
        now = self.clock()
        self._read_packet(now)
        recent_imu = self.imu_angles if now - self.imu_time <= 0.5 else None
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
        if self.reader is not None and self.reader.error:
            source_status += f" | {self.transport.upper()}: {self.reader.error}"
        elif self.transport == "ble" and self.reader is not None:
            source_status += f" | BLE {self.reader.status}"

        stop_requested = False
        if self.pipeline is not None and not self.pipeline.isRunning():
            source_status = "OAK-D pipeline stopped"
            stop_requested = True

        return LiveFusionSnapshot(
            angles=fused,
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
        )

    def accept_camera_observation(self, observation, now: float) -> None:
        """Use the same alignment and camera correction for live and replay data."""
        raw_camera_angles = camera_control_angles(
            observation.get("angles_deg") if isinstance(observation, dict) else None
        )
        self.raw_camera_angles = dict(raw_camera_angles)
        if self.reference_angles is not None:
            for name, value in raw_camera_angles.items():
                origin = self.camera_origin.setdefault(name, value)
                raw_camera_angles[name] = self.reference_angles.get(name, origin) + value - origin
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
                # Correct once per new observation, not once per rendered loop.
                correction_weight = 1.0 - (1.0 - self.camera_weight) ** (
                    min(1.0, max(0.001, camera_dt)) / (1.0 / 30.0)
                )
                if self.transport == "ble" and self.reader is not None:
                    corrected = self.reader.correct_angles(measured, correction_weight)
                    if corrected is not None:
                        self.imu_angles = corrected
                elif now - self.imu_time <= 0.5 and self.imu_angles is not None:
                    self.imu_angles = self.estimator.correct_angles(measured, correction_weight)
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
                snapshot.angles,
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
        self._closed = True
        self._worker_stop.set()
        if self._worker is not None:
            self._worker.join(timeout=5)
            if self._worker.is_alive():
                raise RuntimeError("Camera worker did not stop; input resources are still in use")
        if self.reader is not None:
            self.reader.stop()
        if self.pose is not None:
            self.pose.close()
        if self.hands is not None:
            self.hands.close()
        if self.cup_detector is not None:
            self.cup_detector.close()
        if self.show_monitor and hasattr(self, "cv2"):
            with contextlib.suppress(self.cv2.error):
                self.cv2.destroyWindow(WINDOW)
        if self.pipeline is not None:
            stop = getattr(self.pipeline, "stop", None)
            if callable(stop):
                stop()
