"""Reusable OAK-D and dual-IMU input for the interactive LIMB simulator."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Any


AI_DIR = Path(__file__).resolve().parent
SRC_DIR = AI_DIR.parents[1]
RECORDING_DIR = SRC_DIR / "recording"
for module_dir in (AI_DIR, RECORDING_DIR):
    if str(module_dir) not in sys.path:
        sys.path.insert(0, str(module_dir))

from imu_protocol import extract_imus
from record_oak_pose import FRAME_SIZE, analyze_frame, build_pipeline
from sensor_fusion import (
    DualImuArmEstimator,
    camera_control_angles,
    fuse_control_angles,
)


WINDOW = "LIMB | Live camera monitor"


@dataclass
class LiveFusionSnapshot:
    """Latest control and monitoring values from both live sources."""

    angles: dict[str, float] | None
    sensors: dict[str, dict[str, Any]]
    camera_ready: bool
    imu_ready: bool
    status: str
    stop_requested: bool = False


class SerialPacketReader:
    """Keep only the newest ESP32 packet, following LIMB-HT25's reader."""

    def __init__(self, serial_module, port: str, baud: int) -> None:
        self.serial_module = serial_module
        self.port = port
        self.baud = baud
        self.running = False
        self.packets: queue.Queue[dict[str, object]] = queue.Queue(maxsize=1)
        self.thread: threading.Thread | None = None
        self.error = ""

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def latest(self) -> dict[str, object] | None:
        try:
            return self.packets.get_nowait()
        except queue.Empty:
            return None

    def _read_loop(self) -> None:
        while self.running:
            try:
                with self.serial_module.Serial(
                    self.port, self.baud, timeout=0.25
                ) as device:
                    self.error = ""
                    while self.running:
                        raw = device.readline()
                        if not raw:
                            continue
                        try:
                            import json

                            packet = json.loads(
                                raw.decode("utf-8", errors="ignore").strip()
                            )
                        except (UnicodeDecodeError, ValueError):
                            continue
                        if not isinstance(packet, dict) or not extract_imus(packet):
                            continue
                        if self.packets.full():
                            try:
                                self.packets.get_nowait()
                            except queue.Empty:
                                pass
                        self.packets.put_nowait(packet)
            except (self.serial_module.SerialException, OSError) as error:
                self.error = str(error)
                if self.running:
                    time.sleep(1.0)


def _draw_overlay(frame, status: str, fused: dict[str, float] | None, cv2):
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
        values = (
            f"Shoulder flex {fused.get('shoulder_flexion', 0):.1f}  "
            f"abd {fused.get('shoulder_abduction', 0):.1f}  "
            f"rot {fused.get('shoulder_rotation_proxy', 0):.1f}  "
            f"elbow {fused.get('elbow_flexion', 0):.1f} deg"
        )
    else:
        values = "Waiting for a camera pose or both IMUs"
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
        "C calibrate IMUs   Q stop live control",
        (14, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (174, 187, 207),
        1,
        cv2.LINE_AA,
    )
    return canvas


class LiveSensorInput:
    """Own the live camera window and one dual-IMU serial connection."""

    def __init__(
        self,
        port: str,
        baud: int,
        side: str,
        camera_weight: float,
        depth: bool,
    ) -> None:
        self.port = port
        self.baud = baud
        self.side = side
        self.camera_weight = camera_weight
        self.depth = depth
        self.reader = None
        self.pipeline = None
        self.pose = None
        self.hands = None
        self.depth_queue = None
        self.video_queue = None
        self.intrinsics = None
        self.latest_depth = None
        self.latest_frame = None
        self.camera_status = "Waiting for OAK-D frame"
        self.camera_angles: dict[str, float] = {}
        self.camera_time = 0.0
        self.imu_angles: dict[str, float] | None = None
        self.imu_time = 0.0
        self.previous_imu_time = time.monotonic()
        self.sensors: dict[str, dict[str, Any]] = {}
        self.estimator = DualImuArmEstimator()
        self._closed = False

    def start(self) -> None:
        """Start hardware and show the camera monitor."""
        try:
            import cv2
            import depthai as dai
            import mediapipe as mp
            import numpy as np
            import serial
        except ImportError as error:
            raise RuntimeError(f"Live control dependency missing: {error.name}") from error

        self.cv2 = cv2
        self.mp = mp
        self.np = np
        self.reader = SerialPacketReader(serial, self.port, self.baud)
        self.reader.start()
        try:
            self.pipeline, self.video_queue, self.depth_queue = build_pipeline(
                dai, self.depth
            )
            self.pipeline.start()
            if self.depth:
                calibration = self.pipeline.getDefaultDevice().readCalibration()
                self.intrinsics = calibration.getCameraIntrinsics(
                    dai.CameraBoardSocket.CAM_A, *FRAME_SIZE
                )
            self.pose = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self.hands = mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.moveWindow(WINDOW, 540, 420)
        except Exception:
            self.close()
            raise

    def calibrate(self) -> None:
        self.estimator.reset_calibration()
        self.imu_angles = None
        self.imu_time = 0.0

    def poll(self) -> LiveFusionSnapshot:
        """Advance both sources once and refresh the camera monitor."""
        now = time.monotonic()
        packet = self.reader.latest() if self.reader is not None else None
        if packet is not None:
            self.sensors = extract_imus(packet)
            dt = now - self.previous_imu_time
            self.previous_imu_time = now
            self.imu_angles = self.estimator.update(self.sensors, dt)
            if self.imu_angles is not None:
                self.imu_time = now

        if self.depth_queue is not None:
            depth_message = self.depth_queue.tryGet()
            if depth_message is not None:
                self.latest_depth = depth_message.getFrame()
        frame_message = self.video_queue.tryGet() if self.video_queue is not None else None
        if frame_message is not None:
            frame = frame_message.getCvFrame()
            if frame is not None:
                self.latest_frame, self.camera_status, _sample, observation = analyze_frame(
                    frame,
                    self.latest_depth,
                    self.pose,
                    self.mp,
                    self.cv2,
                    self.np,
                    self.side,
                    self.intrinsics,
                    self.hands,
                )
                measured = camera_control_angles(
                    observation.get("angles_deg")
                    if isinstance(observation, dict)
                    else None
                )
                if measured:
                    self.camera_angles = measured
                    self.camera_time = now

        recent_imu = self.imu_angles if now - self.imu_time <= 0.5 else None
        recent_camera = self.camera_angles if now - self.camera_time <= 0.5 else None
        fused = fuse_control_angles(recent_imu, recent_camera, self.camera_weight)
        source_status = (
            f"Camera {'OK' if recent_camera else '--'} | "
            f"IMUs {'OK' if recent_imu else '--'}"
        )
        if self.reader is not None and self.reader.error:
            source_status += f" | Serial: {self.reader.error}"

        stop_requested = False
        if self.pipeline is not None and not self.pipeline.isRunning():
            source_status = "OAK-D pipeline stopped"
            stop_requested = True
        if self.latest_frame is not None:
            canvas = _draw_overlay(
                self.latest_frame,
                f"{source_status} | {self.camera_status}",
                fused,
                self.cv2,
            )
            self.cv2.imshow(WINDOW, canvas)
        key = self.cv2.waitKey(1) & 0xFF
        if key == ord("c"):
            self.calibrate()
        elif key == ord("q"):
            stop_requested = True
        if (
            hasattr(self.cv2, "getWindowProperty")
            and self.cv2.getWindowProperty(WINDOW, self.cv2.WND_PROP_VISIBLE) < 1
        ):
            stop_requested = True

        return LiveFusionSnapshot(
            angles=fused,
            sensors=self.sensors,
            camera_ready=recent_camera is not None,
            imu_ready=recent_imu is not None,
            status=f"{source_status} | {self.camera_status}",
            stop_requested=stop_requested,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.reader is not None:
            self.reader.stop()
        if self.pose is not None:
            self.pose.close()
        if self.hands is not None:
            self.hands.close()
        if hasattr(self, "cv2"):
            try:
                self.cv2.destroyWindow(WINDOW)
            except self.cv2.error:
                pass
        if self.pipeline is not None:
            stop = getattr(self.pipeline, "stop", None)
            if callable(stop):
                stop()
