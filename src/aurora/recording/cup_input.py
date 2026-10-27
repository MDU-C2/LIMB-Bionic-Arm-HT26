# Majd
"""Capture camera frames independently of optional pose tracking."""

from __future__ import annotations

import contextlib
import threading
import time
from collections import deque
from dataclasses import replace

from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.camera.oak_camera import (
    DEPTH_MATCH_TOLERANCE_NS,
    FRAME_SIZE,
    DepthFrame,
    build_pipeline,
    configure_depthai_runtime,
    device_timestamp_ns,
    matching_depth,
    next_tracking_frame,
    receive_depth,
    receive_frame,
)
from aurora.sensors.camera.oak_camera import CameraFrame as CupCameraFrame

CupDepthFrame = DepthFrame


class CupSensorInput:
    """Keep BLE and raw RGB capture running when tracking is slow or unavailable."""

    def __init__(self, *, side, device, shoulder_twist_axis, wrist_twist_axis, depth=True):
        self.side = side
        self.depth = depth
        self.intrinsics = None
        self.calibration_error = ""
        self.reader = BleSensorReader(
            device,
            shoulder_twist_axis=shoulder_twist_axis,
            wrist_twist_axis=wrist_twist_axis,
            synchronize_clock=True,
            side=side,
        )
        self.cv2 = self.pipeline = self.video_queue = self.depth_queue = None
        self.camera_frame = self.tracking_frame = None
        self.frames = deque(maxlen=75)
        self.observations = deque(maxlen=75)
        self.depth_frames = deque(maxlen=75)
        self.depth_history = deque(maxlen=10)
        self.rgb_history = deque(maxlen=10)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame_event = threading.Event()
        self.threads = []
        self.received_frames = self.tracked_frames = 0
        self.frame_overruns = self.tracking_overruns = 0
        self.camera_sequence_gaps = 0
        self.last_device_sequence = None
        self.received_depth_frames = self.depth_overruns = self.depth_sequence_gaps = 0
        self.last_depth_sequence = None
        self.camera_error = self.tracking_error = ""
        self._stereo_cache_key = self._stereo_cache = None

    @property
    def frame_time_ns(self):
        return self.camera_frame.frame_time_ns if self.camera_frame is not None else 0

    def start(self):
        configure_depthai_runtime()
        import cv2
        import depthai as dai

        self.cv2 = cv2
        # Leave CPU time for BLE, frame receipt and the operator interface.
        cv2.setNumThreads(1)
        try:
            self.pipeline, self.video_queue, self.depth_queue = build_pipeline(
                dai, use_depth=self.depth
            )
            self.pipeline.start()
            if self.depth:
                try:
                    calibration = self.pipeline.getDefaultDevice().readCalibration()
                    self.intrinsics = calibration.getCameraIntrinsics(
                        dai.CameraBoardSocket.CAM_A, *FRAME_SIZE
                    )
                except Exception as error:
                    # Keep the received RGB/depth maps for later analysis.
                    self.calibration_error = str(error)
            self.reader.start()
        except Exception:
            self.close()
            raise

    def start_background(self):
        if self.threads:
            return
        for target in (self._capture, self._track):
            thread = threading.Thread(target=target, daemon=True)
            self.threads.append(thread)
            thread.start()

    def _publish_frame(self, frame):
        with self.lock:
            self.received_frames += 1
            if len(self.frames) == self.frames.maxlen:
                self.frame_overruns += 1
            self.frames.append(frame)
            self.rgb_history.append(frame)
            self.camera_frame = frame
        self.frame_event.set()

    def _capture(self):
        try:
            while not self.stop_event.is_set():
                if not self.pipeline.isRunning():
                    raise RuntimeError("Camera stream stopped")
                if self.depth_queue is not None:
                    depth_message = self.depth_queue.tryGet()
                    if depth_message is not None:
                        self._publish_depth(receive_depth(depth_message))
                message = self.video_queue.tryGet()
                if message is None:
                    self.stop_event.wait(0.002)
                    continue
                image, stamp = receive_frame(message)
                if image is not None:
                    device_sequence = message.getSequenceNum()
                    if self.last_device_sequence is not None:
                        self.camera_sequence_gaps += max(
                            0, device_sequence - self.last_device_sequence - 1
                        )
                    self.last_device_sequence = device_sequence
                    self._publish_frame(
                        CupCameraFrame(
                            self.received_frames + 1,
                            stamp,
                            image,
                            device_frame_sequence=device_sequence,
                            device_time_ns=device_timestamp_ns(message),
                        )
                    )
        except Exception as error:
            self.camera_error = str(error)

    def _publish_depth(self, frame):
        with self.lock:
            if self.last_depth_sequence is not None:
                self.depth_sequence_gaps += max(
                    0, frame.frame_sequence - self.last_depth_sequence - 1
                )
            self.last_depth_sequence = frame.frame_sequence
            self.received_depth_frames += 1
            if len(self.depth_frames) == self.depth_frames.maxlen:
                self.depth_overruns += 1
            self.depth_frames.append(frame)
            self.depth_history.append(frame)
        self.frame_event.set()

    def matching_depth(self, frame):
        """Match by device time, never by whichever map happened to arrive last."""
        with self.lock:
            candidates = list(self.depth_history)
        return matching_depth(frame.device_time_ns, candidates)

    def next_tracking_frame(self, last_sequence):
        """Wait for stereo's delayed map without holding up the raw camera view."""
        with self.lock:
            frames = list(self.rgb_history)
            depths = list(self.depth_history)
        return next_tracking_frame(
            frames, depths, last_sequence, use_depth=self.depth, now_ns=time.monotonic_ns()
        )

    def _track(self):
        pose = hands = cups = None
        try:
            import mediapipe as mp
            import numpy as np

            from aurora.sensors.camera.cup_detection import CupDetector
            from aurora.sensors.camera.hand_tracking import ArmHandTracker
            from aurora.sensors.camera.oak_camera import require_pose_model
            from aurora.sensors.camera.oak_tracking import analyze_frame

            require_pose_model(mp, 1)
            pose = mp.solutions.pose.Pose(
                model_complexity=1, min_detection_confidence=0.5, min_tracking_confidence=0.5
            )
            hands, cups = ArmHandTracker(mp, self.cv2), CupDetector(self.cv2, np)
            last_sequence = -1
            while not self.stop_event.is_set():
                self.frame_event.wait(0.1)
                self.frame_event.clear()
                frame, depth_frame = self.next_tracking_frame(last_sequence)
                if frame is None:
                    continue
                last_sequence = frame.frame_sequence
                image, _, sample, observation = analyze_frame(
                    frame.latest_frame,
                    depth_frame.depth_mm if depth_frame is not None else None,
                    pose,
                    mp,
                    self.cv2,
                    np,
                    self.side,
                    intrinsics=self.intrinsics,
                    hands=hands,
                    cup_detector=cups,
                )
                observation["source_frame_sequence"] = frame.frame_sequence
                observation["rgb_device_time_ns"] = frame.device_time_ns
                if depth_frame is not None:
                    observation.update(
                        {
                            "depth_frame_sequence": depth_frame.frame_sequence,
                            "depth_host_monotonic_ns": depth_frame.frame_time_ns,
                            "depth_device_time_ns": depth_frame.device_time_ns,
                            "depth_time_difference_ns": depth_frame.device_time_ns
                            - frame.device_time_ns,
                        }
                    )
                tracked = replace(
                    frame, latest_frame=image, camera_sample=sample, camera_observation=observation
                )
                with self.lock:
                    self.tracked_frames += 1
                    if len(self.observations) == self.observations.maxlen:
                        self.tracking_overruns += 1
                    self.observations.append(tracked)
                    self.tracking_frame = tracked
        except Exception as error:
            self.tracking_error = str(error)
        finally:
            for tracker in (pose, hands, cups):
                if tracker is not None:
                    with contextlib.suppress(Exception):
                        tracker.close()

    def drain_frames(self):
        with self.lock:
            frames = list(self.frames)
            self.frames.clear()
        return frames

    def drain_tracking(self):
        with self.lock:
            observations = list(self.observations)
            self.observations.clear()
        return observations

    def drain_depth(self):
        with self.lock:
            frames = list(self.depth_frames)
            self.depth_frames.clear()
        return frames

    def camera_metadata(self):
        return {
            "stereo_depth_enabled": self.depth,
            "rgb_size": list(FRAME_SIZE),
            "rgb_transport_format": "NV12",
            "camera_intrinsics": self.intrinsics,
            "camera_intrinsics_source": "device_calibration"
            if self.intrinsics is not None
            else None,
            "camera_calibration_error": self.calibration_error,
            "depth_units": "mm",
            "depth_alignment": "rgb",
            "depth_matching": "nearest_device_timestamp",
            "depth_match_tolerance_ns": DEPTH_MATCH_TOLERANCE_NS,
            "device_timestamp_reference": "oak_device_monotonic_ns",
        }

    def depth_receiving(self):
        with self.lock:
            return bool(
                self.depth_history
                and time.monotonic_ns() - self.depth_history[-1].frame_time_ns <= 1_000_000_000
            )

    def stereo_status(self):
        """Check actual metric arm data, not just whether stereo was selected."""
        import numpy as np

        with self.lock:
            depth = self.depth_history[-1] if self.depth_history else None
            tracked = self.tracking_frame
        now = time.monotonic_ns()
        key = (
            depth.frame_sequence if depth is not None else None,
            tracked.frame_sequence if tracked is not None else None,
            repr(self.intrinsics),
            bool(depth is not None and now - depth.frame_time_ns <= 1_000_000_000),
            bool(tracked is not None and now - tracked.frame_time_ns <= 1_000_000_000),
            self.tracking_error,
        )
        if key == self._stereo_cache_key:
            return dict(self._stereo_cache)
        intrinsics = np.asarray(self.intrinsics, dtype=float)
        calibrated = bool(
            intrinsics.shape == (3, 3)
            and np.isfinite(intrinsics).all()
            and intrinsics[0, 0] > 0
            and intrinsics[1, 1] > 0
        )
        fraction = 0.0
        if depth is not None and hasattr(depth.depth_mm, "shape"):
            values = depth.depth_mm
            if values.ndim == 2 and values.size:
                fraction = float(np.count_nonzero((values > 0) & (values < 3000)) / values.size)
        required = ("left_shoulder", "right_shoulder", f"{self.side}_elbow", f"{self.side}_wrist")
        points = (tracked.camera_sample or {}).get("keypoints_camera_m", {}) if tracked else {}
        arm_valid = bool(
            tracked is not None
            and tracked.device_time_ns is not None
            and time.monotonic_ns() - tracked.frame_time_ns <= 1_000_000_000
            and all(name in points for name in required)
            and abs(tracked.camera_observation.get("depth_time_difference_ns", 1e9))
            <= DEPTH_MATCH_TOLERANCE_NS
        )
        if arm_valid:
            arm_valid = all(
                np.asarray(points[name]).shape == (3,) and np.isfinite(points[name]).all()
                for name in required
            )
        missing_points = [name for name in required if name not in points]
        depth_receiving = self.depth_receiving()
        capture_ready = bool(self.depth and calibrated and depth_receiving and fraction > 0)
        if not self.depth:
            message = "Stereo depth is disabled"
        elif not calibrated:
            message = "Camera calibration could not be read; reconnect the camera"
        elif not depth_receiving:
            message = "Waiting for stereo depth frames"
        elif not fraction:
            message = "No valid depth within 3 m; check the camera view"
        elif self.tracking_error:
            message = "Tracking failed: " + self.tracking_error
        elif tracked is None:
            message = "Starting arm tracking"
        elif now - tracked.frame_time_ns > 1_000_000_000:
            message = "Arm tracking is delayed; keep the subject in view"
        elif "depth_time_difference_ns" not in tracked.camera_observation:
            message = "Waiting for matching RGB and depth frames"
        elif missing_points:
            message = "No 3D point: " + ", ".join(name.replace("_", " ") for name in missing_points)
        elif not arm_valid:
            message = "Arm depth points are invalid"
        else:
            message = "Stereo arm points ready"
        status = {
            "enabled": self.depth,
            "receiving": depth_receiving,
            "intrinsics_available": calibrated,
            "valid_fraction_under_3m": fraction,
            "arm_points_valid": arm_valid,
            "capture_ready": capture_ready,
            "ready": capture_ready and arm_valid,
            "missing_points": missing_points,
            "message": message,
        }
        self._stereo_cache_key, self._stereo_cache = key, status
        return dict(status)

    def capture_health(self):
        with self.lock:
            return {
                "received_rgb_frames": self.received_frames,
                "tracked_rgb_frames": self.tracked_frames,
                "raw_frame_queue_overruns": self.frame_overruns,
                "tracking_queue_overruns": self.tracking_overruns,
                "camera_sequence_gaps": self.camera_sequence_gaps,
                "received_depth_frames": self.received_depth_frames,
                "depth_frame_queue_overruns": self.depth_overruns,
                "depth_sequence_gaps": self.depth_sequence_gaps,
                "camera_error": self.camera_error,
                "tracking_error": self.tracking_error,
            }

    def poll(self):
        # A camera/tracking failure must not discard the remaining BLE data.
        return self.capture_health()

    def close(self):
        self.stop_event.set()
        self.frame_event.set()
        errors = []
        deadline = time.monotonic() + 5.0
        for thread in self.threads:
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
            if thread.is_alive():
                errors.append("Camera worker did not stop; resources remain owned")
        if not errors and self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception as error:
                errors.append(f"Camera pipeline: {error}")
            else:
                self.pipeline = None
        try:
            if self.reader.stop() is False:
                raise RuntimeError("worker did not stop; connection remains owned")
        except Exception as error:
            errors.append(f"Sensor reader: {error}")
        if errors:
            raise RuntimeError("; ".join(errors))
