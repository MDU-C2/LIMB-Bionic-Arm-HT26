# Majd
"""Retain USB bytes, receipt times and partial captures without live hardware."""

from __future__ import annotations

import base64
import csv
import json
import tempfile
import tracemalloc
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from aurora.recording import record_high_rate_serial, record_serial_sensors
from aurora.recording.saved_signals import load_signals


class SerialRecordingTests(unittest.TestCase):
    def test_disconnect_keeps_bytes_samples_and_receipt_before_slow_decoding(self):
        clock = [0.0]
        closed = []

        class SerialFailure(Exception):
            pass

        class SlowBytes(bytes):
            def decode(self, *args, **kwargs):
                clock[0] += 1
                return super().decode(*args, **kwargs)

        messages = [
            SlowBytes(b"EMG,1000,2048\r\n"),
            b"IMU1,1100,1000,0,0,20,30,40\n",
            b"boot \xff\n",
        ]

        class Serial:
            def __init__(self, *args, **kwargs):
                self.index = 0

            def __enter__(self):
                clock[0] += 30
                return self

            def __exit__(self, *args):
                closed.append(True)

            def readline(self):
                clock[0] += 0.01
                if self.index == len(messages):
                    raise SerialFailure("USB disconnected")
                value = messages[self.index]
                self.index += 1
                return value

        args = SimpleNamespace(port="test", baud=115200, duration=2, subject="S01")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with (
                patch.object(record_high_rate_serial, "parse_args", return_value=args),
                patch.object(record_serial_sensors, "create_session_directory", return_value=root),
                patch.object(
                    record_serial_sensors,
                    "time",
                    SimpleNamespace(
                        monotonic=lambda: clock[0], monotonic_ns=lambda: round(clock[0] * 1e9)
                    ),
                ),
                patch.dict(
                    "sys.modules",
                    {"serial": SimpleNamespace(Serial=Serial, SerialException=SerialFailure)},
                ),
            ):
                self.assertEqual(record_high_rate_serial.main(), 1)
            self.assertEqual(closed, [True])
            records = [
                json.loads(line) for line in (root / "serial.jsonl").read_text().splitlines()
            ]
            self.assertEqual([base64.b64decode(row["raw_base64"]) for row in records], messages)
            self.assertEqual(records[0]["host_monotonic_ns"], 30_010_000_000)
            with (root / "recording.csv").open() as source:
                rows = list(csv.DictReader(source, delimiter=";"))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["host_monotonic_ns"], str(records[0]["host_monotonic_ns"]))
            self.assertEqual(rows[1]["imu1_timestamp_us"], "1100")
            self.assertEqual(rows[1]["emg_raw"], "")
            meta = json.loads((root / "meta.json").read_text())
            self.assertFalse(meta["capture_complete"])
            self.assertIn("disconnected", meta["capture_error"])
            self.assertAlmostEqual(meta["duration_seconds"], 1.04)
            self.assertEqual(meta["samples"], {"emg": 1, "imu1": 1, "imu2": 0})
            self.assertTrue(load_signals(root / "recording.csv").panels)

    def test_export_memory_stays_bounded_and_preserves_each_received_measurement(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with (root / "serial.jsonl").open("w") as output:
                for index in range(20_000):
                    output.write(
                        json.dumps(
                            dict(
                                text=f"EMG,{index * 2500},2048",
                                host_monotonic_ns=1_000_000_000 + index * 2_500_000,
                                host_time="receipt",
                            )
                        )
                        + "\n"
                    )
            tracemalloc.start()
            try:
                result = record_high_rate_serial.export_samples(root)
                _current, peak = tracemalloc.get_traced_memory()
            finally:
                tracemalloc.stop()
            self.assertEqual(result["rows"], 20_000)
            self.assertLess(peak, 2_000_000)
            self.assertTrue(result["sample_data_complete"])
            with (root / "recording.csv").open() as source:
                self.assertEqual(sum(1 for _ in csv.DictReader(source, delimiter=";")), 20_000)

    def test_duplicate_reordering_and_device_clock_reset_remain_in_the_export(self):
        stamps = [1_000_000, 1_000_000, 999_990, 1_010_000, 10, 20]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with (root / "serial.jsonl").open("w") as output:
                for index, stamp in enumerate(stamps):
                    output.write(
                        json.dumps(
                            dict(
                                text=f"EMG,{stamp},2048",
                                host_monotonic_ns=index,
                                host_time="receipt",
                            )
                        )
                        + "\n"
                    )
                output.write(
                    json.dumps(
                        dict(text="IMU2,50,nan,0,0,0,0,0", host_monotonic_ns=7, host_time="receipt")
                    )
                    + "\n"
                )
            result = record_high_rate_serial.export_samples(root)
            self.assertEqual(result["rows"], len(stamps))
            self.assertEqual(
                result["sample_quality"],
                dict(
                    duplicates=1, reordered_samples=1, device_clock_resets=1, invalid_sensor_lines=1
                ),
            )
            self.assertFalse(result["sample_data_complete"])
            with (root / "recording.csv").open() as source:
                rows = list(csv.DictReader(source, delimiter=";"))
            self.assertEqual([int(row["timestamp_us"]) for row in rows], stamps)
            self.assertEqual([int(row["device_clock_epoch"]) for row in rows], [0, 0, 0, 0, 1, 1])


if __name__ == "__main__":
    unittest.main()
