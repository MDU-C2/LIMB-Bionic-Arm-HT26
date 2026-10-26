# Majd
"""Check grip cues, labels and signal features without hardware."""

from __future__ import annotations

import argparse
import csv
import io
import json
import os
import struct
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from aurora.recording.emg_features import export_features
from aurora.recording.emg_trials import GripTrial
from aurora.recording.record_ble_sensors import record
from aurora.sensors.ble_protocol import HIGH_RATE_EMG_HEADER, SENSOR_UUIDS
from aurora.sensors.clock_sync import SYNC_RESPONSE


class GripTrialTests(unittest.TestCase):
    def test_repeat_limits(self):
        for count in (0, 21):
            with self.assertRaises(ValueError):
                GripTrial(count)

    def test_delayed_cue_uses_actual_time_and_keeps_every_phase(self):
        trial = GripTrial(1)
        self.assertIn("Subject relaxes", trial.advance(1_000_000_000))
        self.assertIn("Subject closes", trial.advance(5_000_000_000))
        self.assertEqual(trial.segments[1]["start_s"], 4)
        self.assertIsNone(trial.advance(5_200_000_000))
        self.assertIn("holding", trial.advance(5_500_000_000))
        self.assertIn("Subject opens", trial.advance(8_500_000_000))
        self.assertIn("Subject relaxes", trial.advance(9_000_000_000))
        self.assertIn("finished", trial.advance(12_000_000_000))
        self.assertTrue(trial.complete)
        self.assertEqual(
            [item["phase"] for item in trial.segments], ["rest", "grip", "hold", "release", "rest"]
        )

    def test_interrupted_trial_keeps_cue_times_and_reports_missing_data(self):
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            trial = GripTrial(1)
            trial.advance(1_000_000_000)
            summary = trial.save(Path(temporary), 2_000_000_000)
            labels = json.loads((Path(temporary) / "emg_trial.json").read_text())
        self.assertFalse(summary["complete"])
        self.assertFalse(summary["features_saved"])
        self.assertEqual(labels["segments"][0]["end_s"], 1)

    def test_features_keep_amplitude_and_exclude_mixed_phase_windows(self):
        from aurora.recording.emg_processing import main

        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            origin = 1_000_000_000
            source = session / "emg.csv"
            labels_file = session / "emg_trial.json"
            labels_file.write_text(
                json.dumps(
                    {
                        "start_host_monotonic_ns": origin,
                        "segments": [
                            {"trial": 1, "phase": "rest", "start_s": 0, "end_s": 2},
                            {"trial": 1, "phase": "hold", "start_s": 2, "end_s": 4},
                        ],
                    }
                )
            )
            times = np.arange(16000) / 4000
            raw = 2048 + np.where(times < 2, 2, 20) * np.sin(2 * np.pi * 100 * times)
            with source.open("w", newline="") as output:
                writer = csv.writer(output)
                writer.writerow(("device_time", "channel", "value", "synced_host_monotonic_ns"))
                writer.writerows(
                    (index * 250, 0, value, origin + index * 250_000)
                    for index, value in enumerate(raw)
                )
            original = source.read_bytes()
            metadata_file = session / "meta.json"
            original_metadata = '{"grip_trial": {"original": true}}'
            metadata_file.write_text(original_metadata, encoding="utf-8")
            result = export_features(source, labels_file)
            self.assertAlmostEqual(result["hold_rest_ratio"], 10, delta=0.05)
            with (session / result["features"]).open() as output:
                rows = list(csv.DictReader(output))
            for row in rows:
                timestamp = float(row["timestamp_us"]) / 1e6
                boundary = 0 if row["phase"] == "rest" else 2
                self.assertGreaterEqual(timestamp, boundary + 0.05)
                self.assertLess(timestamp + 0.2, boundary + 1.951)
                self.assertEqual(row["session"], session.name)
            with (
                patch(
                    "sys.argv",
                    ["emg_processing.py", "--file", str(source), "--labels", str(labels_file)],
                ),
                redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(), 0)
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual(metadata_file.read_text(encoding="utf-8"), original_metadata)
            labels = json.loads(labels_file.read_text())
            labels["segments"][1]["start_s"] = 1.5
            labels_file.write_text(json.dumps(labels))
            with self.assertRaisesRegex(ValueError, "non-overlapping"):
                export_features(source, labels_file)
            labels["segments"][1]["start_s"] = 2
            labels_file.write_text(json.dumps(labels))
            with source.open() as recorded:
                data = list(csv.DictReader(recorded))
            fieldnames = tuple(data[0])
            for row in data[8000:]:
                row["device_time"] = str(int(row["device_time"]) + 250)
            with source.open("w", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(data)
            with self.assertRaisesRegex(ValueError, "regularly spaced"):
                export_features(source, labels_file)


class GripRecordingTests(unittest.IsolatedAsyncioTestCase):
    async def test_invalid_grip_setup_does_not_open_ble(self):
        for overrides in (
            {"dataset_label": "1"},
            {"preview": True},
            {"sensors": ["imu"]},
            {"grip_repeats": 0},
            {"duration": 2},
        ):
            args = dict(
                device="AURORA",
                address="test",
                scan_timeout=1,
                subject="S01",
                duration=0,
                dataset_label="",
                sensors=["emg"],
                grip_trial=True,
                grip_repeats=1,
            )
            args.update(overrides)
            with patch("bleak.BleakClient") as connect, redirect_stdout(io.StringIO()):
                self.assertEqual(await record(argparse.Namespace(**args)), 2)
                connect.assert_not_called()

    async def test_guided_ble_recording_saves_all_phases_and_rest_hold_comparison(self):
        now = 0.0
        sample_index = 0
        sequence = 0
        origin = 1_000_000_000_000
        client = SimpleNamespace(is_connected=True, callbacks={})

        class FakeClient:
            async def __aenter__(self):
                return client

            async def __aexit__(self, *_args):
                return None

        async def notify(uuid, callback):
            client.callbacks[uuid] = callback

        client.start_notify = notify

        class FakeSync:
            def __init__(self, _client, clock):
                self.clock = clock

            async def start(self):
                sequence, _ = self.clock.make_request(origin)
                self.clock.handle_response(SYNC_RESPONSE.pack(1, 0, sequence, origin, 0, 0), origin)
                return True

            async def close(self):
                return None

        async def sleep(seconds):
            nonlocal now, sample_index, sequence
            count = round(seconds * 4000)
            while count:
                size = min(40, count)
                times = np.arange(sample_index, sample_index + size) / 4000
                amplitude = np.where((times >= 6.1) & (times < 9.5), 20, 2)
                raw = np.rint(2048 + amplitude * np.sin(2 * np.pi * 100 * times)).astype(int)
                payload = HIGH_RATE_EMG_HEADER.pack(
                    0x0101, sequence, sample_index * 250, 250, size, sample_index
                )
                client.callbacks[SENSOR_UUIDS["emg"]](
                    None, payload + struct.pack(f"<{size}H", *raw)
                )
                sample_index += size
                sequence += 1
                count -= size
            now += seconds

        clock = SimpleNamespace(
            monotonic=lambda: now, monotonic_ns=lambda: origin + round(now * 1e9)
        )
        args = argparse.Namespace(
            device="AURORA",
            address="test",
            scan_timeout=1,
            subject="S01",
            duration=0,
            dataset_label="",
            sensors=["emg"],
            grip_trial=True,
            grip_repeats=1,
        )
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            with (
                patch.dict(os.environ, {"AURORA_OUTPUT_DIR": temporary}),
                patch("bleak.BleakClient", lambda _: FakeClient()),
                patch("aurora.recording.record_ble_sensors.BleClockSyncClient", FakeSync),
                patch("aurora.recording.record_ble_sensors.time", clock),
                patch("aurora.recording.record_ble_sensors.asyncio.sleep", sleep),
            ):
                result = await record(args)
            session = next(Path(temporary).iterdir())
            metadata = json.loads((session / "meta.json").read_text())
            summary = metadata["grip_trial"]
            self.assertEqual(result, 0)
            self.assertTrue(summary["complete"])
            self.assertTrue(summary["features_saved"])
            self.assertTrue(all(count > 0 for count in summary["windows_by_phase"].values()))
            self.assertGreater(summary["hold_rest_ratio"], 5)
            self.assertTrue((session / "emg.csv").is_file())
            self.assertTrue((session / "packets.jsonl").is_file())


if __name__ == "__main__":
    unittest.main()
