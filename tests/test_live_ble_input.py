# Majd
"""Check BLE live control without an ESP32, camera, or simulator window."""

from __future__ import annotations

import asyncio
import struct
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

from aurora.gui.simulation_tabs import SimulationTabsMixin
from aurora.recording.record_ble_sensors import (
    HIGH_RATE_EMG_HEADER,
    HIGH_RATE_IMU_PACKET,
    SENSOR_UUIDS,
)
from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.emg_protocol import EmgActivationEstimator
from aurora.sensors.fusion import DualImuArmEstimator, interactive_control_targets
from aurora.sensors.live_input import LiveSensorInput
from aurora.simulation.live_control import main as launch_live


def emg_packet(values, sequence=0, period_us=250):
    return HIGH_RATE_EMG_HEADER.pack(
        0x101, sequence, sequence * 10_000, period_us, len(values), 0
    ) + struct.pack(f"<{len(values)}H", *values)


def imu_packet(
    mask=3,
    timestamp=1_000_000,
    wrist=(0, 0, 1000, 0, 0, 0),
    shoulder=(0, 0, 1000, 0, 0, 0),
    sequence=None,
):
    if sequence is None:
        sequence = timestamp // 10_000
    return HIGH_RATE_IMU_PACKET.pack(
        0x100 | mask, sequence, timestamp, timestamp, *shoulder, *wrist
    )


class LiveBleTests(unittest.TestCase):
    def test_axial_correction_uses_the_readers_clock(self):
        clock = [1000.0]
        reader = BleSensorReader(clock=lambda: clock[0])
        for index in range(50):
            reader.notification("imu", imu_packet(timestamp=1_000_000 + index * 10_000))
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=999999.0):
            self.assertIsNotNone(reader.correct_rotation(-20, 1.0))
            clock[0] += 0.6
            self.assertIsNone(reader.correct_rotation(-30, 1.0))

    def test_camera_corrects_twist_once_and_gyro_continues_between_frames(self):
        reader = BleSensorReader()
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.0):
            for sample in range(50):
                reader.notification("imu", imu_packet(timestamp=1_000_000 + sample * 10_000))
            controller = LiveSensorInput()
        controller.reader = reader
        controller.cv2 = SimpleNamespace(waitKey=lambda _delay: -1, imshow=lambda *_args: None)
        controller.mp = controller.np = None
        frame = SimpleNamespace(getCvFrame=lambda: object())
        controller.video_queue = SimpleNamespace(
            tryGet=Mock(side_effect=[frame, None, frame, None])
        )
        observation = {"angles_deg": {"shoulder_rotation_proxy": -20.0}}
        with (
            patch(
                "aurora.sensors.live_input.analyze_frame",
                return_value=(object(), "OK", None, observation),
            ),
            patch("aurora.sensors.live_input._draw_overlay", return_value=object()),
        ):
            with patch("aurora.sensors.live_input.time.monotonic", return_value=100.01):
                snapshot = controller.poll()
                self.assertAlmostEqual(snapshot.angles["shoulder_rotation_proxy"], -20.0)
                self.assertEqual(snapshot.rotation_source, "IMU + camera")
            with patch("aurora.sensors.live_input.time.monotonic", return_value=100.02):
                reader.notification(
                    "imu",
                    imu_packet(
                        timestamp=1_500_000,
                        shoulder=(0, 0, 1000, 0, 3000, 0),
                        wrist=(0, 0, 1000, 0, 3000, 0),
                    ),
                )
                snapshot = controller.poll()
                self.assertAlmostEqual(snapshot.angles["shoulder_rotation_proxy"], -20.3)
                self.assertEqual(snapshot.angles["wrist_rotation"], 0.0)
            with patch("aurora.sensors.live_input.time.monotonic", return_value=100.03):
                snapshot = controller.poll()
                self.assertGreater(snapshot.angles["shoulder_rotation_proxy"], -20.3)
                self.assertLess(snapshot.angles["shoulder_rotation_proxy"], -20.0)
            with patch("aurora.sensors.live_input.time.monotonic", return_value=100.6):
                reader.notification(
                    "imu",
                    imu_packet(
                        timestamp=1_510_000,
                        shoulder=(0, 0, 1000, 0, 3000, 0),
                        wrist=(0, 0, 1000, 0, 3000, 0),
                    ),
                )
                snapshot = controller.poll()
                self.assertFalse(snapshot.camera_ready)
                self.assertEqual(snapshot.rotation_source, "IMU")
                self.assertLess(snapshot.angles["shoulder_rotation_proxy"], -20.3)
        controller.calibrate()
        controller.latest_frame = controller.video_queue = None
        self.assertIsNone(controller.poll().angles)

    def test_binary_packets_reach_simulator_arm_and_grip(self):
        reader = BleSensorReader()
        reader.imu_estimator = DualImuArmEstimator(alpha=0)
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.0):
            reader.notification("imu", imu_packet())
            reader.notification("imu", imu_packet(timestamp=1_010_000, wrist=(0, 1000, 0, 0, 0, 0)))
            for sequence in range(50):
                reader.notification("emg", emg_packet([2048] * 40, sequence))
            reader.notification("emg", emg_packet([2300, 1800] * 20, 50))
            controller = LiveSensorInput()
            controller.reader = reader
            controller.cv2 = SimpleNamespace(waitKey=lambda _delay: -1)
            snapshot = controller.poll()
        self.assertTrue(snapshot.imu_ready)
        self.assertFalse(snapshot.camera_ready)
        self.assertEqual(set(snapshot.sensors), {"shoulder", "wrist"})
        self.assertEqual(interactive_control_targets(snapshot.angles)["elbow_x"], 60.0)
        self.assertEqual(snapshot.grip_source, "EMG")
        self.assertGreater(snapshot.hand_curl, 0)
        self.assertIsNone(reader.decoder.raw_file)
        self.assertEqual(reader.decoder.files, {})

    def test_all_emg_samples_use_the_device_period(self):
        reader = BleSensorReader()
        reference = EmgActivationEstimator(2000)
        for sequence in range(50):
            reader.notification("emg", emg_packet([2048] * 40, sequence))
            for _sample in range(40):
                reference.update(2048, 0.00025)
        self.assertTrue(reader.emg_estimator.calibrated)
        values = [2200, 1900] * 20
        reader.notification("emg", emg_packet(values, 50))
        for value in values:
            reference.update(value, 0.00025)
        self.assertAlmostEqual(reader.emg_estimator.envelope, reference.envelope)
        self.assertEqual(reader.latest()["emg_raw"], 1900)

    def test_camera_grip_reaches_control_with_an_active_resting_emg_stream(self):
        reader = BleSensorReader()
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.0):
            for sequence in range(50):
                reader.notification("emg", emg_packet([8] * 40, sequence))
            controller = LiveSensorInput(grip_source="camera")
            controller.reader = reader
            controller.cv2 = SimpleNamespace(waitKey=lambda _delay: -1)
            controller.hand_time = 100.0
            controller.hand_curl = 1.0
            snapshot = controller.poll()
            self.assertEqual(snapshot.emg_activation, 0.0)
            self.assertEqual(snapshot.hand_curl, 1.0)
            self.assertEqual(snapshot.grip_source, "camera")
            controller.hand_curl = 0.0
            self.assertEqual(controller.poll().hand_curl, 0.0)

    def test_emg_does_not_keep_old_imus_alive(self):
        reader = BleSensorReader()
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.0):
            reader.notification("imu", imu_packet())
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.6):
            reader.notification("emg", emg_packet([2048] * 40))
            controller = LiveSensorInput()
            controller.reader = reader
            controller.cv2 = SimpleNamespace(waitKey=lambda _delay: -1)
            snapshot = controller.poll()
        self.assertFalse(snapshot.imu_ready)
        self.assertIsNone(snapshot.angles)
        self.assertTrue(all(not sensor["connected"] for sensor in snapshot.sensors.values()))
        self.assertEqual(snapshot.emg_raw, 2048)

    def test_graphs_keep_received_samples_bounded_and_hide_stale_streams(self):
        reader = BleSensorReader()
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.0):
            for sequence in range(25):
                reader.notification("emg", emg_packet(list(range(40)), sequence))
            reader.notification("imu", imu_packet())
            data = reader.graph_data()
            self.assertEqual(data["emg"], list(range(40)) * 20)
            self.assertEqual(data["emg_period"], 0.00025)
            self.assertEqual(data["imus"]["shoulder"], [(0, 0, 1, 0, 0, 0)])
            data["emg"].clear()
            self.assertEqual(len(reader.graph_data()["emg"]), 800)
            reader.notification("imu", imu_packet(mask=1, timestamp=1_010_000))
            self.assertEqual(reader.graph_data()["imus"]["wrist"], [])
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.6):
            self.assertEqual(reader.graph_data()["emg"], [])
            self.assertEqual(reader.graph_data()["imus"]["shoulder"], [])
        with reader._lock:
            reader._clear_state()
        self.assertFalse(reader._emg_history)

    def test_imus_do_not_keep_old_emg_alive(self):
        reader = BleSensorReader()
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.0):
            for sequence in range(50):
                reader.notification("emg", emg_packet([2048] * 40, sequence))
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.6):
            reader.notification("imu", imu_packet())
            controller = LiveSensorInput()
            controller.reader = reader
            controller.cv2 = SimpleNamespace(waitKey=lambda _delay: -1)
            snapshot = controller.poll()
        self.assertTrue(snapshot.imu_ready)
        self.assertIsNone(snapshot.emg_activation)
        self.assertIsNone(snapshot.emg_raw)
        self.assertEqual(snapshot.grip_source, "--")

    def test_wrist_only_is_monitor_only_and_role_masks_are_respected(self):
        reader = BleSensorReader()
        reader.notification("imu", imu_packet(mask=2))
        self.assertEqual(set(reader.latest()["imus"]), {"wrist"})
        self.assertIsNone(reader.latest()["imu_angles"])
        reader.notification("imu", imu_packet(mask=0, timestamp=1_010_000))
        self.assertEqual(reader.latest()["imus"], {})

    def test_old_imu_packets_do_not_change_pose_or_extend_freshness(self):
        clock = [100.0]
        reader = BleSensorReader(clock=lambda: clock[0])
        reader.imu_estimator = DualImuArmEstimator(alpha=0)
        reader.notification("imu", imu_packet(sequence=10))
        reader.notification(
            "imu", imu_packet(sequence=11, timestamp=1_010_000, wrist=(0, 1000, 0, 0, 0, 0))
        )
        pose = reader.latest()["imu_angles"]
        history = reader.graph_data()["imus"]
        clock[0] += 0.4
        for sequence in (11, 10):
            reader.notification("imu", imu_packet(sequence=sequence, timestamp=1_020_000))
        self.assertEqual(reader.latest()["imu_angles"], pose)
        self.assertEqual(reader.graph_data()["imus"], history)
        self.assertEqual(reader.imu_time, 100.0)
        clock[0] += 0.2
        self.assertEqual(reader.graph_data()["imus"]["wrist"], [])
        reader.notification("imu", imu_packet(sequence=12, timestamp=1_030_000))
        self.assertNotEqual(reader.latest()["imu_angles"], pose)
        self.assertEqual(reader.imu_time, clock[0])

    def test_imu_sequence_wrap_is_a_new_sample(self):
        reader = BleSensorReader()
        reader.notification("imu", imu_packet(sequence=0xFFFFFFFF))
        reader.notification("imu", imu_packet(sequence=0, timestamp=1_010_000))
        self.assertEqual(len(reader.graph_data()["imus"]["shoulder"]), 2)

    def test_gyro_integration_uses_device_time(self):
        reader = BleSensorReader()
        with patch("aurora.sensors.ble_input.time.monotonic", return_value=100.0):
            reader.notification("imu", imu_packet())
            with patch.object(
                reader.imu_estimator, "update", wraps=reader.imu_estimator.update
            ) as update:
                reader.notification("imu", imu_packet(timestamp=1_010_000))
        self.assertAlmostEqual(update.call_args.args[1], 0.01)

    def test_arm_alignment_preserves_emg_calibration(self):
        reader = BleSensorReader()
        reader.notification("imu", imu_packet())
        for sequence in range(50):
            reader.notification("emg", emg_packet([2048] * 40, sequence))
        controller = LiveSensorInput()
        controller.reader = reader
        controller.calibrate()
        self.assertTrue(reader.emg_estimator.calibrated)
        self.assertEqual(reader.latest()["emg_activation"], 0.0)
        self.assertIsNone(reader.latest()["imu_angles"])
        controller.calibrate_emg()
        self.assertFalse(reader.emg_estimator.calibrated)
        self.assertIsNone(reader.latest()["emg_activation"])
        reader.notification("emg", emg_packet([2048] * 40, 50))
        self.assertAlmostEqual(reader.emg_estimator.calibration_progress, 0.02)

    def test_emg_only_recalibration_keeps_imu_alignment(self):
        reader = BleSensorReader()
        reader.notification("imu", imu_packet())
        previous_angles = reader.latest()["imu_angles"]
        for sequence in range(50):
            reader.notification("emg", emg_packet([2048] * 40, sequence))
        reader.calibrate_emg()
        self.assertFalse(reader.emg_estimator.calibrated)
        self.assertIsNone(reader.latest()["emg_activation"])
        self.assertEqual(reader.latest()["imu_angles"], previous_angles)

    def test_stop_cancels_a_pending_scan(self):
        scanning = threading.Event()

        async def scan(*_args, **_kwargs):
            scanning.set()
            await asyncio.Future()

        reader = BleSensorReader()
        with patch("bleak.BleakScanner.find_device_by_filter", side_effect=scan):
            reader.start()
            try:
                self.assertTrue(scanning.wait(timeout=2))
            finally:
                reader.stop()
        self.assertFalse(reader.thread.is_alive())
        self.assertEqual(reader.status, "Stopped")

    def test_connected_stop_allows_disconnect_cleanup_to_finish(self):
        reader = BleSensorReader()
        reader.status = "Connected"
        reader._loop = Mock()
        reader._task = Mock()
        reader.thread = Mock()
        reader.stop(timeout=0)
        reader.status = "Reconnecting"
        reader.stop(timeout=0)
        reader._loop.call_soon_threadsafe.assert_not_called()
        self.assertTrue(reader._stop.is_set())

    def test_gui_launches_ble_without_any_serial_settings(self):
        def setting(value):
            return SimpleNamespace(get=lambda: value)

        application = SimpleNamespace(
            simulation_python=Path(sys.executable),
            recording_ble_device=setting("AURORA"),
            fusion_camera_weight=setting("0.25"),
            processes={},
            _stop_serial_dashboard=Mock(),
            imu_monitor=None,
            dynamic_simulation=setting(False),
            recording_camera_side=setting("left"),
            fusion_depth=setting(False),
            fusion_grip_source=setting("Camera"),
            fusion_shoulder_twist_axis=setting("y"),
            fusion_wrist_twist_axis=setting("-y"),
            _start_program=Mock(),
        )
        SimulationTabsMixin.start_live_control(application)
        command = application._start_program.call_args.args[2]
        self.assertEqual(command[command.index("--transport") + 1], "ble")
        self.assertEqual(command[command.index("--device") + 1], "AURORA")
        self.assertNotIn("--port", command)
        self.assertEqual(command[command.index("--table-width-cm") + 1], "60")
        self.assertEqual(command[command.index("--table-depth-cm") + 1], "60")
        self.assertEqual(command[command.index("--grip-source") + 1], "camera")
        self.assertEqual(command[command.index("--shoulder-twist-axis") + 1], "y")
        self.assertEqual(command[command.index("--wrist-twist-axis") + 1], "-y")

    def test_compatibility_launcher_defaults_to_ble_and_keeps_serial_commands(self):
        for arguments, expected in (([], "ble"), (["--port", "COM5"], "serial")):
            with (
                patch.object(sys, "argv", ["live_control.py", *arguments]),
                patch("aurora.simulation.live_control.subprocess.call", return_value=0) as launch,
            ):
                self.assertEqual(launch_live(), 0)
                command = launch.call_args.args[0]
                self.assertEqual(command[command.index("--transport") + 1], expected)


class BleConnectionTests(unittest.IsolatedAsyncioTestCase):
    def test_camera_reference_preserves_displayed_pose(self):
        controller = LiveSensorInput(camera_weight=0)
        controller.mp = controller.np = Mock()
        controller.cv2 = SimpleNamespace(waitKey=lambda _delay: -1)
        controller.video_queue = Mock()
        frame = controller.video_queue.tryGet.return_value
        frame.getCvFrame.return_value = object()
        measured = {"shoulder_flexion": 35, "elbow_flexion": 40, "shoulder_abduction": 8}
        with patch(
            "aurora.sensors.live_input.analyze_frame",
            return_value=(None, "tracked", None, {"angles_deg": measured}),
        ):
            controller.poll()
            controller.calibrate(
                {"shoulder_flexion": 45, "elbow_flexion": 45, "shoulder_abduction": 5}
            )
            snapshot = controller.poll()
        self.assertEqual(snapshot.angles["shoulder_flexion"], 45)
        self.assertEqual(snapshot.angles["elbow_flexion"], 45)
        self.assertEqual(snapshot.angles["shoulder_abduction"], 5)

    async def test_disconnect_clears_old_values_and_reconnects(self):
        reader = BleSensorReader(address="test-device")
        attempts = []
        callbacks = {}

        class Client:
            is_connected = False

            async def __aenter__(self):
                attempts.append(self)
                if len(attempts) == 2:
                    self.test_cleared = reader.latest()["imus"] == {}
                    reader._stop.set()
                return self

            async def __aexit__(self, *_args):
                return False

            async def start_notify(self, uuid, callback):
                callbacks[uuid] = callback
                if uuid == SENSOR_UUIDS["imu"]:
                    callback(None, imu_packet())
                else:
                    callback(None, emg_packet([2048] * 40))

        with (
            patch("bleak.BleakClient", side_effect=lambda _target: Client()),
            patch("aurora.sensors.ble_input.asyncio.sleep", new_callable=AsyncMock),
        ):
            await reader._connect_loop()
        self.assertEqual(len(attempts), 2)
        self.assertTrue(attempts[1].test_cleared)
        self.assertEqual(set(callbacks), {SENSOR_UUIDS["imu"], SENSOR_UUIDS["emg"]})
        self.assertEqual(reader.latest()["imus"], {})
        self.assertIsNone(reader.latest()["emg_activation"])

    async def test_missing_emg_characteristic_does_not_stop_imu(self):
        reader = BleSensorReader(address="test-device")

        class Client:
            is_connected = True

            async def __aenter__(self):
                return self

            async def __aexit__(self, *_args):
                return False

            async def start_notify(self, uuid, callback):
                if uuid == SENSOR_UUIDS["emg"]:
                    raise RuntimeError("characteristic missing")
                callback(None, imu_packet())

        async def stop_after_subscription(_delay):
            self.assertEqual(reader.status, "Connected")
            self.assertIn("EMG", reader.error)
            self.assertEqual(set(reader.latest()["imus"]), {"shoulder", "wrist"})
            reader._stop.set()

        with (
            patch("bleak.BleakClient", return_value=Client()),
            patch("aurora.sensors.ble_input.asyncio.sleep", side_effect=stop_after_subscription),
        ):
            await reader._connect_loop()


if __name__ == "__main__":
    unittest.main()
