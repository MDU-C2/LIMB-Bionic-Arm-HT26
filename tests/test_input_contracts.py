"""Input rejection, ownership and cleanup contracts without hardware."""

from __future__ import annotations

import base64
import csv
import json
import struct
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from aurora.recording.packet_writer import PacketWriter
from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.ble_protocol import HIGH_RATE_EMG_HEADER, HIGH_RATE_IMU_PACKET, BleRecorder
from aurora.sensors.clock_sync import SYNC_RESPONSE, ClockSynchronizer
from aurora.sensors.emg_packets import CURRENT_EMG_PACKET, decode_emg
from aurora.sensors.imu_packets import decode_imu
from aurora.sensors.imu_protocol import ImuStreamDecoder, connected_roles
from aurora.sensors.live_input import LiveSensorInput
from aurora.simulation.stepping import FixedStepClock


class InputContractsTests(unittest.TestCase):
    def test_invalid_current_adc_values_are_counted_and_retained_only_as_raw_packets(self):
        for invalid in (4096, 65535):
            for payload in (
                CURRENT_EMG_PACKET.pack(0x101, 1, 1000, invalid),
                HIGH_RATE_EMG_HEADER.pack(0x100, 1, 1000, 250, 3, 0)
                + struct.pack("<3H", 0, invalid, 4095),
            ):
                with self.subTest(value=invalid, size=len(payload)):
                    with self.assertRaisesRegex(ValueError, "12-bit"):
                        decode_emg(payload)
                    with tempfile.TemporaryDirectory() as temporary:
                        root = Path(temporary)
                        recorder = BleRecorder(root)
                        try:
                            recorder.handle("emg", payload)
                            self.assertEqual(recorder.packet_health()["emg"]["decode_errors"], 1)
                        finally:
                            recorder.close()
                        raw = json.loads((root / "packets.jsonl").read_text())
                        self.assertEqual(base64.b64decode(raw["data_base64"]), payload)
                        with (root / "emg.csv").open() as source:
                            self.assertEqual(list(csv.DictReader(source)), [])
        valid = HIGH_RATE_EMG_HEADER.pack(0x100, 1, 1000, 250, 2, 0)
        self.assertEqual(decode_emg(valid + struct.pack("<2H", 0, 4095)).channels, ((0, 4095),))
        # The single-sample adapter has a presence bit; its unused field is not a reading.
        self.assertEqual(decode_emg(CURRENT_EMG_PACKET.pack(0x100, 1, 1000, 65535)).channels, ())

    def test_wire_saturated_gyros_remain_raw_but_do_not_integrate_or_command_control(self):
        clock = [100.0]
        live = LiveSensorInput(show_monitor=False, clock=lambda: clock[0])
        live.reader = BleSensorReader(clock=live.clock)
        live.accept_camera_observation(
            {"angles_deg": {"elbow_flexion": 90, "shoulder_flexion": 0}}, clock[0]
        )
        live.calibrate({"elbow_flexion": 45})
        for index in range(60):
            clock[0] += 0.01
            live.reader.notification(
                "imu",
                HIGH_RATE_IMU_PACKET.pack(
                    0x103, index, index * 10_000, index * 10_000, *([0, 0, 1000, 0, 0, 0] * 2)
                ),
            )
        self.assertTrue(live.poll().calibration_valid)
        before = dict(live.reader.imu_estimator.last_angles)
        payload = HIGH_RATE_IMU_PACKET.pack(
            0x103, 60, 600_000, 600_000, 0, 0, 1000, 0, 0, 32767, 0, 0, 1000, 0, 0, -32768
        )
        self.assertEqual(decode_imu(payload).gyro_saturated_roles, (0, 1))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            live.reader.start_recording(root)
            clock[0] += 0.01
            live.reader.notification("imu", payload)
            state = live.poll()
            self.assertFalse(state.calibration_valid)
            self.assertTrue(state.imu_ready)
            self.assertIsNone(state.angles)
            self.assertIn("wire limit", state.status)
            self.assertEqual(live.reader.imu_estimator.last_angles, before)
            self.assertTrue(all(sensor["gyro_saturated"] for sensor in state.sensors.values()))
            health = live.reader.stop_recording()
            self.assertEqual(health["packet_health"]["imu"]["gyro_saturation_samples"], 2)
            with (root / "imu.csv").open() as source:
                rows = list(csv.DictReader(source))
            self.assertEqual([row["gyro_saturated"] for row in rows], ["1", "1"])
            self.assertEqual([float(row["gyro_z"]) for row in rows], [327.67, -327.68])
        live.close()

    def test_stop_retains_a_received_packet_while_processing_is_slow(self):
        reader = BleSensorReader()
        payload = bytearray(
            HIGH_RATE_EMG_HEADER.pack(0x100, 1, 1000, 250, 1, 0) + struct.pack("<H", 2048)
        )
        original = bytes(payload)
        entered, release = threading.Event(), threading.Event()
        decode = reader.decoder.handle

        def slow_processing(*args, **kwargs):
            entered.set()
            release.wait(3)
            decode(*args, **kwargs)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reader.start_recording(root)
            with patch.object(reader.decoder, "handle", slow_processing):
                worker = threading.Thread(target=reader.notification, args=("emg", payload))
                worker.start()
                try:
                    self.assertTrue(entered.wait(1))
                    with self.assertRaisesRegex(RuntimeError, "still processing"):
                        reader.stop_recording(timeout=0)
                    with self.assertRaisesRegex(RuntimeError, "already recording"):
                        reader.start_recording(root / "next")
                    payload[:] = b"changed after receipt"
                finally:
                    release.set()
                    worker.join(2)
            self.assertFalse(worker.is_alive())
            health = reader.stop_recording()
            packets = [
                json.loads(line) for line in (root / "packets.jsonl").read_text().splitlines()
            ]
            self.assertEqual(len(packets), 1)
            self.assertEqual(base64.b64decode(packets[0]["data_base64"]), original)
            self.assertEqual(health["packet_counts"]["emg"], 1)
            self.assertEqual(health["raw_packet_queue_overruns"], 0)

    def test_start_does_not_absorb_a_packet_received_before_the_trial(self):
        reader = BleSensorReader()
        entered, release = threading.Event(), threading.Event()
        decode = reader.decoder.handle

        def slow_processing(*args, **kwargs):
            entered.set()
            release.wait(3)
            decode(*args, **kwargs)

        payload = HIGH_RATE_EMG_HEADER.pack(0x100, 1, 1000, 250, 1, 0) + struct.pack("<H", 2048)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with patch.object(reader.decoder, "handle", slow_processing):
                worker = threading.Thread(target=reader.notification, args=("emg", payload))
                worker.start()
                try:
                    self.assertTrue(entered.wait(1))
                    reader.start_recording(root)
                finally:
                    release.set()
                    worker.join(2)
            reader.stop_recording()
            self.assertEqual((root / "packets.jsonl").read_text(), "")

    def test_truncated_current_emg_cannot_masquerade_as_fixed_legacy(self):
        truncated = HIGH_RATE_EMG_HEADER.pack(0x100, 123, 1000, 250, 40, 0)
        truncated += struct.pack("<31H", *([2048] * 31))
        self.assertEqual(len(truncated), 84)
        with self.assertRaisesRegex(ValueError, "payload size"):
            decode_emg(truncated)
        legacy = struct.pack("<40HI", *([512] + [2048] * 39), 123)
        with self.assertRaises(ValueError):
            decode_emg(legacy)
        decoded = decode_emg(legacy, legacy_fixed=True)
        self.assertEqual(decoded.sequence, 123)
        self.assertEqual(decoded.channels[0], (512, *([2048] * 39)))
        self.assertIsNone(decoded.first_time_us)

    def test_new_serial_connection_cannot_publish_previous_connections_emg(self):
        clock = [1000.0]
        live = LiveSensorInput(transport="serial", show_monitor=False, clock=lambda: clock[0])
        live.feed_serial(
            {
                "connection_generation": 1,
                "host_monotonic_ns": 1_000_000_000_000,
                "sample_time_us": 1000,
                "updated_sensor": "emg",
                "emg": {"connected": True, "adc_raw": 2048},
            }
        )
        self.assertEqual(live.poll().emg_raw, 2048)
        clock[0] += 0.1
        live.feed_serial(
            {"connection_generation": 2, "host_monotonic_ns": 1_000_100_000_000, "imus": {}}
        )
        self.assertIsNone(live.poll().emg_raw)
        self.assertIsNone(live.poll().emg_activation)
        self.assertFalse(live.poll().calibration_valid)

    def test_held_ble_timestamps_do_not_reset_the_anatomical_reference(self):
        reader = BleSensorReader()
        reader.calibrate(reference_angles={"elbow_flexion": 90, "shoulder_flexion": 0})
        for sequence, stamp in ((0, 1_000_000), (1, 1_000_000), (2, 1_010_000)):
            reader.notification(
                "imu",
                HIGH_RATE_IMU_PACKET.pack(
                    0x103, sequence, stamp, stamp, *([0, 0, 1000, 0, 0, 0] * 2)
                ),
            )
            if sequence == 0:
                offsets = dict(reader.imu_estimator.tilt_offsets)
            self.assertEqual(reader.imu_estimator.tilt_offsets, offsets)
        self.assertAlmostEqual(reader.latest()["imu_angles"]["elbow_flexion"], 90)

    def test_imu_gap_holds_reference_and_requires_alignment_without_integrating_missing_motion(
        self,
    ):
        clock = [1000.0]
        reader = BleSensorReader(clock=lambda: clock[0])
        reader.calibrate(reference_angles={"elbow_flexion": 90, "shoulder_flexion": 0})

        def send(sequence, stamp, speed):
            reader.notification(
                "imu",
                HIGH_RATE_IMU_PACKET.pack(
                    0x103, sequence, stamp, stamp, *([0, 0, 1000, 0, 0, speed] * 2)
                ),
            )

        send(0, 1_000_000, 0)
        offsets = dict(reader.imu_estimator.tilt_offsets)
        clock[0] += 1
        send(1, 2_000_000, 10_000)
        clock[0] += 0.01
        send(2, 2_010_000, 0)
        state = reader.latest()
        self.assertTrue(state["imu_motion_gap"])
        self.assertEqual(reader.imu_estimator.tilt_offsets, offsets)
        self.assertAlmostEqual(state["imu_angles"]["elbow_flexion"], 90)
        self.assertAlmostEqual(state["imu_angles"]["shoulder_abduction"], 0)
        reader.calibrate(reset_emg=False, reference_angles={"elbow_flexion": 90})
        self.assertFalse(reader.latest()["imu_motion_gap"])

    def test_serial_channel_receipt_is_not_replaced_by_later_decode_time(self):
        decoder = ImuStreamDecoder()
        with patch("aurora.sensors.imu_protocol.time.monotonic_ns", return_value=100_000_000_000):
            packet = decoder.feed("IMU1,1000,0,0,1000,0,0,0", host_monotonic_ns=10_000_000_000)
        self.assertEqual(packet["imus"]["shoulder"]["host_monotonic_ns"], 10_000_000_000)

    def test_recorder_releases_open_files_after_partial_startup(self):
        opened = []
        original = Path.open

        def open_file(path, *args, **kwargs):
            file = original(path, *args, **kwargs)
            opened.append(file)
            return file

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "emg.csv").mkdir()
            with patch.object(Path, "open", open_file), self.assertRaises(OSError):
                BleRecorder(root)
            self.assertEqual(len(opened), 2)
            self.assertTrue(all(file.closed for file in opened))

    def test_continuous_sequences_do_not_hide_overlapping_emg_sample_times(self):
        reader = BleSensorReader()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reader.start_recording(root)
            for sequence, stamp in enumerate((1000, 5000, 21000)):
                payload = HIGH_RATE_EMG_HEADER.pack(0x100, sequence, stamp, 250, 40, 0)
                reader.notification("emg", payload + struct.pack("<40H", *([2048] * 40)))
            health = reader.stop_recording()["packet_health"]["emg"]
            self.assertEqual(health["missing_packets"], 0)
            self.assertEqual(health["sample_time_gaps"], 2)
            self.assertEqual(len(reader._emg_history), 80)
            with (root / "emg.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 120)
            self.assertTrue(all(row["connection_generation"] == "0" for row in rows))
            self.assertEqual(len((root / "packets.jsonl").read_text().splitlines()), 3)

    def test_simulator_releases_independent_resources_when_live_cleanup_fails(self):
        import io

        from aurora.simulation.interactive import runtime

        simulation = object.__new__(runtime.InteractiveSimulator)
        simulation.live_sensor_input = Mock()
        simulation.live_sensor_input.close.side_effect = RuntimeError("stuck inference")
        simulation.sensor_window = Mock()
        simulation.telemetry_file = io.StringIO()
        simulation.torque_csv_file = io.StringIO()
        telemetry, torque = simulation.telemetry_file, simulation.torque_csv_file
        window = simulation.sensor_window
        with patch.object(runtime.pygame, "quit"), patch.object(runtime, "p") as physics:
            physics.isConnected.return_value = True
            with self.assertRaisesRegex(RuntimeError, "stuck inference"):
                simulation.close()
            self.assertTrue(telemetry.closed)
            self.assertTrue(torque.closed)
            window.destroy.assert_called_once()
            physics.disconnect.assert_called_once()
            simulation.live_sensor_input.close.side_effect = None
            simulation.close()
        self.assertIsNone(simulation.live_sensor_input)
        window.destroy.assert_called_once()

    def test_camera_pipeline_configuration_failure_releases_its_owner(self):
        from types import SimpleNamespace

        from aurora.sensors.camera.oak_camera import build_pipeline

        pipeline = Mock()
        pipeline.create.side_effect = RuntimeError("camera node failed")
        dai = SimpleNamespace(Pipeline=lambda: pipeline, node=SimpleNamespace(Camera=object()))
        with self.assertRaisesRegex(RuntimeError, "camera node failed"):
            build_pipeline(dai)
        pipeline.stop.assert_called_once()

    def test_each_ble_role_expires_without_reusing_its_old_angle(self):
        for held in ("shoulder", "wrist"):
            clock = [1000.0]
            reader = BleSensorReader(clock=lambda clock=clock: clock[0])

            def send(sequence, shoulder, wrist, reader=reader):
                reader.notification(
                    "imu",
                    HIGH_RATE_IMU_PACKET.pack(
                        0x103, sequence, shoulder, wrist, *([0, 0, 1000, 0, 0, 0] * 2)
                    ),
                )

            send(0, 1_000_000, 1_000_000)
            clock[0] += 0.6
            send(
                1,
                1_000_000 if held == "shoulder" else 1_600_000,
                1_000_000 if held == "wrist" else 1_600_000,
            )
            state = reader.latest()
            self.assertNotIn(held, state["imus"])
            if held == "shoulder":
                self.assertIsNone(state["imu_angles"])
            else:
                self.assertNotIn("elbow_flexion", state["imu_angles"] or {})
                self.assertNotIn("wrist_rotation", state["imu_angles"] or {})
            self.assertEqual(reader.graph_data()["imus"][held], [])

    def test_serial_held_gyro_is_integrated_once_and_expires_without_new_packets(self):
        import copy

        clock = [1000.0]
        live = LiveSensorInput(transport="serial", show_monitor=False, clock=lambda: clock[0])
        live._worker = Mock()
        packet = {
            "connection_generation": 1,
            "host_monotonic_ns": round(clock[0] * 1e9),
            "imus": {
                role: {
                    "connected": True,
                    "sample_time_us": 1_000_000,
                    "host_monotonic_ns": round(clock[0] * 1e9),
                    "accel_g": {"x": 0, "y": 0, "z": 1},
                    "gyro_dps": {"x": 0, "y": 0, "z": 0},
                }
                for role in ("shoulder", "wrist")
            },
        }
        live.feed_serial(copy.deepcopy(packet))
        clock[0] += 0.1
        for sensor in packet["imus"].values():
            sensor["sample_time_us"] += 100_000
            sensor["host_monotonic_ns"] = round(clock[0] * 1e9)
            sensor["gyro_dps"]["z"] = 100
        live.feed_serial(copy.deepcopy(packet))
        angle = live.imu_angles["shoulder_abduction"]
        self.assertGreater(angle, 9)
        packet["updated_sensor"] = "emg"
        for _ in range(20):
            live.feed_serial(copy.deepcopy(packet))
        self.assertEqual(live.imu_angles["shoulder_abduction"], angle)
        clock[0] += 0.6
        state = live.poll()
        self.assertFalse(state.imu_ready)
        self.assertIsNone(state.angles)

    def test_device_clock_reset_starts_unsynchronized_epoch_in_raw_recording(self):
        clock = [1000.0]
        reader = BleSensorReader(clock=lambda: clock[0])
        with tempfile.TemporaryDirectory() as temporary:
            reader.start_recording(Path(temporary))
            for sequence, stamp in ((100, 1_000_000), (0, 1000)):
                reader.notification(
                    "imu",
                    HIGH_RATE_IMU_PACKET.pack(
                        0x103, sequence, stamp, stamp, *([0, 0, 1000, 0, 0, 0] * 2)
                    ),
                )
                clock[0] += 1
            health = reader.stop_recording()
            packets = [
                json.loads(line)
                for line in (Path(temporary) / "packets.jsonl").read_text().splitlines()
            ]
            self.assertEqual([packet["connection_generation"] for packet in packets], [0, 1])
            self.assertEqual(health["device_clock_resets"], 1)
            self.assertFalse(reader.clock_sync.ready)
            self.assertIn("clock reset", reader.error)

    def test_fixed_steps_keep_fractional_time_and_catch_up_without_losing_time(self):
        regular, delayed = FixedStepClock(), FixedStepClock()
        regular_steps = sum(regular.advance(1 / 120) for _ in range(120))
        delayed_steps = sum(delayed.advance(value) for value in (0.003, 0.997))
        while delayed.pending_s >= delayed.period_s - 1e-12:
            delayed_steps += delayed.advance(0)
        self.assertEqual(regular_steps, 60)
        self.assertEqual(delayed_steps, regular_steps)
        self.assertAlmostEqual(delayed.wall_time_s, 1.0)
        self.assertLess(delayed.pending_s, 1e-10)

    def test_protocol_adapters_reject_malformed_v1_and_keep_explicit_legacy(self):
        for period, count, actual in ((0, 40, 40), (250, 41, 41), (250, 40, 39)):
            payload = HIGH_RATE_EMG_HEADER.pack(0x100, 1, 1000, period, count, 0)
            with self.assertRaises(ValueError):
                decode_emg(payload + struct.pack(f"<{actual}H", *([2048] * actual)))
        legacy = struct.pack("<I3H", 7, 2000, 2001, 2002)
        with self.assertRaises(ValueError):
            decode_emg(legacy)
        self.assertEqual(decode_emg(legacy, legacy_prefix=True).channels, ((2000, 2001, 2002),))
        with self.assertRaises(ValueError):
            decode_imu(struct.pack("<HIQ12h", 0x203, 7, 1000, *([0] * 12)))

    def test_slow_writer_preserves_receipt_times_and_reports_exact_overflow(self):
        entered, release = threading.Event(), threading.Event()
        original = BleRecorder.handle

        def delayed(recorder, *args, **kwargs):
            entered.set()
            release.wait(2)
            return original(recorder, *args, **kwargs)

        def payload(sequence):
            return HIGH_RATE_EMG_HEADER.pack(
                0x100, sequence, 1000 + sequence * 250, 250, 1, 0
            ) + struct.pack("<H", 2048)

        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(BleRecorder, "handle", delayed),
        ):
            writer = PacketWriter(
                Path(temporary), clock_sync=None, connection_generation=3, capacity=2
            )
            try:
                writer.handle("emg", payload(0), host_monotonic_ns=123)
                self.assertTrue(entered.wait(1))
                for sequence in range(1, 5):
                    writer.handle("emg", payload(sequence), host_monotonic_ns=123 + sequence)
                self.assertEqual(writer.queue_overruns, 2)
            finally:
                release.set()
                writer.close()
            rows = [
                json.loads(line)
                for line in (Path(temporary) / "packets.jsonl").read_text().splitlines()
            ]
            self.assertEqual([row["host_monotonic_ns"] for row in rows], [123, 124, 125])
            self.assertTrue(all(row["connection_generation"] == 3 for row in rows))
            with (Path(temporary) / "emg.csv").open(newline="") as source:
                self.assertEqual(len(list(csv.DictReader(source))), 3)
            self.assertIn("incomplete", writer.summary()["write_error"])

    def test_clock_snapshot_cannot_change_after_later_fit(self):
        clock = ClockSynchronizer()
        request, _ = clock.make_request(1_000_000_000)
        clock.handle_response(
            SYNC_RESPONSE.pack(1, 0, request, 1_000_000_000, 1000, 1001), 1_001_000_000
        )
        snapshot = clock.snapshot()
        mapped = snapshot.to_host_ns(2000)
        request, _ = clock.make_request(2_000_000_000)
        clock.handle_response(
            SYNC_RESPONSE.pack(1, 0, request, 2_000_000_000, 2000, 2001), 2_001_000_000
        )
        self.assertEqual(snapshot.to_host_ns(2000), mapped)
        self.assertNotEqual(clock.to_host_ns(2000), mapped)

    def test_unknown_high_rate_version_is_not_legacy_data(self):
        samples = []
        recorder = BleRecorder(None, sample_sink=lambda *sample: samples.append(sample))
        payload = HIGH_RATE_EMG_HEADER.pack(2 << 8, 123, 1000, 250, 40, 0)
        recorder.handle("emg", payload + struct.pack("<40H", *([2048] * 40)))
        self.assertEqual(recorder.decode_errors["emg"], 1)
        self.assertEqual(samples, [])
        self.assertEqual(recorder.last_sequences, {})

    def test_serial_emg_does_not_refresh_cached_imu_channels(self):
        decoder = ImuStreamDecoder()
        decoder.feed("IMU1,1000,0,0,1000,0,0,0")
        decoder.feed("IMU2,1100,0,0,1000,0,0,0")
        packet = decoder.feed("EMG,100001100,2048")
        self.assertEqual(connected_roles(packet), ())

    def test_serial_channels_keep_their_own_sample_times(self):
        decoder = ImuStreamDecoder()
        decoder.feed("IMU1,1000,0,0,1000,0,0,0")
        packet = decoder.feed("IMU2,1100,0,0,1000,0,0,0")
        self.assertEqual(packet["imus"]["shoulder"]["sample_time_us"], 1000)
        self.assertEqual(packet["imus"]["wrist"]["sample_time_us"], 1100)

    def test_poll_serializes_state_updates(self):
        live = LiveSensorInput(show_monitor=False)
        live._worker = Mock()
        live.reader = Mock()
        live.reader.latest.return_value = None
        finished = threading.Event()

        def poll():
            live.poll()
            finished.set()

        with live._source_lock:
            thread = threading.Thread(target=poll)
            thread.start()
            self.assertFalse(finished.wait(0.05))
            live.reader.latest.assert_not_called()
        thread.join(1)
        self.assertTrue(finished.is_set())

    def test_stuck_camera_worker_can_be_retried_without_closing_its_resources(self):
        live = LiveSensorInput(show_monitor=False)
        live._worker = Mock()
        live._worker.is_alive.return_value = True
        live.pose, live.pipeline = Mock(), Mock()
        with self.assertRaisesRegex(RuntimeError, "worker"):
            live.close()
        self.assertFalse(live._closed)
        live.pose.close.assert_not_called()
        live.pipeline.stop.assert_not_called()
        pose, pipeline = live.pose, live.pipeline
        live._worker.is_alive.return_value = False
        live.close()
        pose.close.assert_called_once()
        pipeline.stop.assert_called_once()
        self.assertTrue(live._closed)

    def test_independent_cleanup_survives_tracker_failure_and_retries(self):
        live = LiveSensorInput(show_monitor=False)
        live.pose, live.hands, live.pipeline = Mock(), Mock(), Mock()
        pose, hands, pipeline = live.pose, live.hands, live.pipeline
        pose.close.side_effect = RuntimeError("tracker failure")
        with self.assertRaisesRegex(RuntimeError, "tracker failure"):
            live.close()
        hands.close.assert_called_once()
        pipeline.stop.assert_called_once()
        self.assertFalse(live._closed)
        pose.close.side_effect = None
        live.close()
        self.assertTrue(live._closed)
        hands.close.assert_called_once()
        pipeline.stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
