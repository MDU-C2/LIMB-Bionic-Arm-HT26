# Majd
"""Verify high-rate serial alignment and BLE packet decoding."""

from __future__ import annotations

import asyncio
import csv
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "recording"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "gui"))

from ble_dataset import LabeledBleCapture
from firmware_workspace import source_files
from high_rate_serial import merge_samples, parse_sensor_line, save_csv
from imu_protocol import ImuStreamDecoder
from plot_sensor_recording import load_recording
from record_ble_sensors import HIGH_RATE_EMG_HEADER, HIGH_RATE_IMU_PACKET, BleRecorder
from record_ble_sensors import record as record_ble


class HighRateIntegrationTests(unittest.TestCase):
    def test_disconnect_stops_recording_and_preserves_metadata(self) -> None:
        client = SimpleNamespace(
            is_connected=True,
            start_notify=AsyncMock(),
        )
        context = AsyncMock()
        context.__aenter__.return_value = client
        sync = SimpleNamespace(start=AsyncMock(return_value=False), close=AsyncMock())

        async def disconnect(_seconds):
            client.is_connected = False

        args = SimpleNamespace(
            sensors=["imu"],
            preview=False,
            duration=0.0,
            scan_timeout=1.0,
            dataset_label="",
            device="LIMBServer",
            address="test-device",
            subject="test",
        )
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            with (
                patch("bleak.BleakClient", return_value=context),
                patch("record_ble_sensors.BleClockSyncClient", return_value=sync),
                patch("record_ble_sensors.create_session_directory", return_value=session),
                patch("record_ble_sensors.asyncio.sleep", side_effect=disconnect),
                self.assertRaisesRegex(RuntimeError, "BLE disconnected"),
            ):
                asyncio.run(record_ble(args))
            metadata = json.loads((session / "meta.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["subscribed_sensors"], ["imu"])
            self.assertEqual(metadata["packet_health"]["imu"]["received_packets"], 0)
            sync.close.assert_awaited_once()

    def test_ble_duration_starts_after_connection_and_subscription(self) -> None:
        elapsed = [100.0]

        class Client:
            is_connected = True

            async def __aenter__(self):
                elapsed[0] += 30.0
                return self

            async def __aexit__(self, *_exception):
                return False

            async def start_notify(self, _uuid, _callback):
                return None

        async def wait(seconds):
            elapsed[0] += seconds

        args = SimpleNamespace(
            sensors=["imu", "emg"],
            preview=False,
            duration=2.0,
            scan_timeout=1.0,
            dataset_label="",
            device="LIMBServer",
            address="test-device",
            subject="test",
        )
        sync = SimpleNamespace(start=AsyncMock(return_value=False), close=AsyncMock())
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            with (
                patch("bleak.BleakClient", return_value=Client()),
                patch("record_ble_sensors.BleClockSyncClient", return_value=sync),
                patch("record_ble_sensors.create_session_directory", return_value=session),
                patch("record_ble_sensors.time.monotonic", side_effect=lambda: elapsed[0]),
                patch("record_ble_sensors.asyncio.sleep", side_effect=wait) as sleeps,
            ):
                self.assertEqual(asyncio.run(record_ble(args)), 0)
            metadata = json.loads((session / "meta.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["duration_seconds"], 2.0)
            self.assertEqual(metadata["subscribed_sensors"], ["imu", "emg"])
            self.assertEqual(sleeps.call_count, 2)

    def test_serial_monitor_combines_tagged_imu_and_emg_lines(self) -> None:
        decoder = ImuStreamDecoder()
        decoder.feed("IMU1,1000,1000,0,0,1000,0,0")
        packet = decoder.feed("EMG,1250,2048")
        assert packet is not None
        self.assertEqual(packet["imus"]["shoulder"]["accel_g"]["x"], 1.0)
        self.assertEqual(packet["imus"]["shoulder"]["gyro_dps"]["x"], 1.0)
        self.assertEqual(packet["emg"]["adc_raw"], 2048)
        self.assertEqual(packet["updated_sensor"], "emg")

    def test_firmware_workspace_includes_component_and_headers(self) -> None:
        project = Path(__file__).resolve().parents[1] / "firmware" / "high_rate_emg_imu"
        inputs = {path.relative_to(project).as_posix() for path in source_files(project)}
        self.assertIn("main/main.c", inputs)
        self.assertIn("main/CMakeLists.txt", inputs)
        self.assertIn("include/config.h", inputs)

    def test_serial_parser_and_alignment_preserve_unmatched_samples(self) -> None:
        self.assertEqual(parse_sensor_line("EMG,1000,2048"), ("emg", 1000, (2048,)))
        self.assertEqual(parse_sensor_line("1250,2047"), ("emg", 1250, (2047,)))
        self.assertEqual(parse_sensor_line("EMG,-250,2048"), ("emg", -250, (2048,)))
        self.assertIsNone(parse_sensor_line("IMU1,1000,nan,0,0,0,0,0"))
        imu = parse_sensor_line("IMU1,1100,1000,0,0,250,0,0")
        assert imu is not None
        rows = merge_samples(
            {
                "emg": [(1000, (2048,)), (1250, (2047,))],
                "imu1": [(imu[1], imu[2])],
                "imu2": [(9000, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))],
            }
        )
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["imu1_timestamp_us"], 1100)
        self.assertEqual(rows[2]["imu2_timestamp_us"], 9000)
        self.assertEqual(rows[2]["timestamp_s"], 0.008)

    def test_csv_round_trip_and_plot_loader(self) -> None:
        rows = merge_samples({"emg": [(1000, (2010,))], "imu1": [], "imu2": []})
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "recording.csv"
            save_csv(path, rows)
            self.assertIn(";", path.read_text(encoding="utf-8").splitlines()[0])
            self.assertEqual(load_recording(path)[0]["emg_raw"], 2010.0)

    def test_ble_packets_write_each_sample_at_its_own_time(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            recorder = BleRecorder(Path(temporary))
            emg = HIGH_RATE_EMG_HEADER.pack(0x0100, 7, 1_000, 250, 3, 0)
            recorder.handle("emg", bytearray(emg + struct.pack("<3H", 100, 200, 300)))
            imu = HIGH_RATE_IMU_PACKET.pack(
                0x0103,
                8,
                1_050,
                1_100,
                1000,
                0,
                0,
                2500,
                0,
                0,
                0,
                1000,
                0,
                0,
                2500,
                0,
            )
            recorder.handle("imu", bytearray(imu))
            recorder.close()
            with (Path(temporary) / "emg.csv").open(newline="", encoding="utf-8") as source:
                emg_rows = list(csv.DictReader(source))
            with (Path(temporary) / "imu.csv").open(newline="", encoding="utf-8") as source:
                imu_rows = list(csv.DictReader(source))
            self.assertEqual([int(row["device_time"]) for row in emg_rows], [1000, 1250, 1500])
            self.assertEqual([row["value"] for row in emg_rows], ["100", "200", "300"])
            self.assertEqual([int(row["device_time"]) for row in imu_rows], [1050, 1100])
            self.assertEqual(imu_rows[0]["accel_x"], "1.0")
            self.assertEqual(imu_rows[0]["gyro_x"], "25.0")

    def test_single_imu_mask_keeps_wrist_role(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            received = []
            recorder = BleRecorder(
                Path(temporary), sample_sink=lambda *sample: received.append(sample)
            )
            packet = HIGH_RATE_IMU_PACKET.pack(
                0x0102,
                4,
                0,
                20_000,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                1000,
                0,
                0,
                2500,
                0,
            )
            recorder.handle("imu", bytearray(packet))
            recorder.close()
            with (Path(temporary) / "imu.csv").open(newline="", encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["sensor"], "1")
            self.assertEqual(rows[0]["device_time"], "20000")
            self.assertEqual(received[0][2][0], [])
            self.assertEqual(received[0][2][1][0][1], 1.0)

    def test_dataset_window_accepts_variable_emg_packet_sizes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            capture = LabeledBleCapture(Path(temporary), "1", "subject")
            capture.start()
            for sequence, count in enumerate([40] * 9 + [16, 24]):
                capture.add_packet("emg", sequence, [list(range(count))])
            self.assertEqual(len(capture.windows["emg"][0]), 1)
            self.assertEqual(len(capture.windows["emg"][0][0][1]), 400)


if __name__ == "__main__":
    unittest.main()
