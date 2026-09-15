"""Check labeled BLE CSVs against the old LIMB capture layout."""

from __future__ import annotations

from contextlib import redirect_stdout
import argparse
import asyncio
import csv
import io
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "recording"))

from ble_dataset import LabeledBleCapture
from record_ble_sensors import BleRecorder, record, SENSOR_UUIDS


class LabeledBleCaptureTests(unittest.TestCase):
    def test_old_single_sensor_packets_make_complete_capture(self) -> None:
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            session = Path(temporary)
            capture = LabeledBleCapture(session, "1", "S01")
            recorder = BleRecorder(session, capture.add_packet)
            capture.start()
            for sequence in range(800):
                recorder.handle(
                    "emg",
                    bytearray(struct.pack("<40HI", *range(40), sequence)),
                )
                recorder.handle(
                    "imu",
                    bytearray(struct.pack("<9fI", *range(9), sequence)),
                )
            recorder.close()
            capture.save()

            self.assertTrue(capture.complete)
            raw_root = session / "raw_data" / "S01"
            emg = next((raw_root / "EMG").glob("*.csv"))
            imu = next((raw_root / "IMU").glob("*.csv"))
            with emg.open(newline="", encoding="utf-8") as source:
                emg_rows = list(csv.reader(source))
            with imu.open(newline="", encoding="utf-8") as source:
                imu_rows = list(csv.reader(source))
            self.assertEqual(emg_rows[0][:3], ["Raw_Label", "Timestamp", "v0"])
            self.assertEqual(len(emg_rows), 81)
            self.assertTrue(all(len(row) == 402 for row in emg_rows))
            self.assertEqual(imu_rows[0][:3], ["Raw_Label", "Timestamp", "accel_x"])
            self.assertEqual(len(imu_rows), 801)

            segments = session / "labeled_data" / "S01" / "segmented_emg"
            sizes = {}
            for path in segments.glob("*.csv"):
                with path.open(newline="", encoding="utf-8") as source:
                    rows = list(csv.reader(source))
                self.assertEqual(rows[0][:3], ["Label", "Timestamp", "v0"])
                sizes[path.stem.split("_")[-1]] = len(rows) - 1
            self.assertEqual(sorted(sizes.values()), [20, 20, 40])
            self.assertEqual(len(list(segments.glob("*.csv"))), 3)

    def test_dropped_packet_keeps_capture_incomplete(self) -> None:
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            capture = LabeledBleCapture(Path(temporary), "1", "S01")
            capture.start()
            for sequence in range(10):
                if sequence == 5:
                    continue
                capture.add_packet("emg", sequence, [list(range(40))])
            capture.save()
            self.assertFalse(capture.complete)
            self.assertEqual(capture.windows["emg"].get(0), [])
            self.assertGreater(capture.sequence_gaps["emg"], 0)
            self.assertFalse((Path(temporary) / "labeled_data").exists())

    def test_dual_sensor_packets_keep_both_channels(self) -> None:
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            session = Path(temporary)
            capture = LabeledBleCapture(session, "1", "S01")
            recorder = BleRecorder(session, capture.add_packet)
            capture.start()
            for sequence in range(10):
                recorder.handle(
                    "emg",
                    bytearray(struct.pack("<HIQ80H", 1, sequence, 0, *range(80))),
                )
                recorder.handle(
                    "imu",
                    bytearray(struct.pack("<HIQ12h", 1, sequence, 0, *range(12))),
                )
            recorder.close()
            capture.save()
            self.assertEqual(len(capture.windows["emg"][0]), 1)
            self.assertEqual(len(capture.windows["emg"][1]), 1)
            self.assertEqual(len(capture.windows["imu"][0]), 1)
            self.assertEqual(len(capture.windows["imu"][1]), 1)
            self.assertEqual(len(list((session / "raw_data" / "S01" / "EMG").glob("*.csv"))), 2)


class LabeledBleConnectionTests(unittest.IsolatedAsyncioTestCase):
    async def test_mock_ble_session_stops_and_writes_metadata(self) -> None:
        class FakeClient:
            callbacks = {}

            def __init__(self, _device):
                self.callbacks = {}

            async def __aenter__(self):
                return self

            async def __aexit__(self, *_args):
                return None

            async def start_notify(self, uuid, callback):
                self.callbacks[uuid] = callback

        class FakeScanner:
            @staticmethod
            async def find_device_by_name(_name, timeout):
                return "mock-limb-server"

        client = None
        sleep_calls = 0
        real_sleep = asyncio.sleep

        async def fake_sleep(_seconds):
            nonlocal sleep_calls, client
            sleep_calls += 1
            if 4 <= sleep_calls <= 11:
                first_sequence = (sleep_calls - 4) * 100
                for sequence in range(first_sequence, first_sequence + 100):
                    client.callbacks[SENSOR_UUIDS["emg"]](
                        None, bytearray(struct.pack("<40HI", *range(40), sequence))
                    )
                    client.callbacks[SENSOR_UUIDS["imu"]](
                        None, bytearray(struct.pack("<9fI", *range(9), sequence))
                    )
            await real_sleep(0)

        def create_client(device):
            nonlocal client
            client = FakeClient(device)
            return client

        args = argparse.Namespace(
            device="LIMBServer", address="", scan_timeout=1.0,
            subject="S01", duration=0.0, dataset_label="1",
        )
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            with patch.dict(os.environ, {"AURORA_OUTPUT_DIR": temporary}), \
                 patch("bleak.BleakClient", create_client), \
                 patch("bleak.BleakScanner", FakeScanner), \
                 patch("record_ble_sensors.asyncio.sleep", fake_sleep):
                result = await record(args)
            self.assertEqual(result, 0)
            session = next(Path(temporary).iterdir())
            meta = json.loads((session / "meta.json").read_text(encoding="utf-8"))
            self.assertTrue(meta["labeled_capture"]["complete"])
            self.assertEqual(meta["labeled_capture"]["windows_by_sensor"]["emg"]["0"], 80)
            self.assertEqual(len(list((session / "labeled_data" / "S01" / "segmented_emg").glob("*.csv"))), 3)


if __name__ == "__main__":
    unittest.main()
