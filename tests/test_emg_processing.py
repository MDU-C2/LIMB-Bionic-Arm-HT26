# Majd
"""Check the recorded EMG pipeline without connecting any sensors."""

from __future__ import annotations

import csv
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from aurora.recording.emg_processing import (
    load_emg,
    main,
    preprocess_emg,
    sample_rate,
    window_features,
)
from aurora.recording.plot_sensor_recording import main as plot_main
from aurora.recording.plot_sensor_recording import make_figure


class EmgProcessingTests(unittest.TestCase):
    def test_filters_remove_offset_and_noise_while_preserving_in_band_signal(self) -> None:
        rate = 4000
        time = np.arange(rate * 4) / rate
        raw = 2048 + sum(
            100 * np.sin(2 * np.pi * frequency * time) for frequency in (5, 50, 100, 800)
        )
        filtered = preprocess_emg(raw, rate)
        middle = slice(rate, -rate)
        self.assertLess(abs(filtered[middle].mean()), 0.1)
        for frequency, lower, upper in ((5, 0, 1), (50, 0, 5), (100, 95, 105), (800, 0, 2)):
            amplitude = abs(
                2 * np.mean(filtered[middle] * np.sin(2 * np.pi * frequency * time[middle]))
            )
            with self.subTest(frequency=frequency):
                self.assertGreaterEqual(amplitude, lower)
                self.assertLess(amplitude, upper)

    def test_nyquist_limit_is_not_silently_changed_for_usb_data(self) -> None:
        raw = np.full(400, 2048.0)
        with self.assertRaisesRegex(ValueError, "Nyquist"):
            preprocess_emg(raw, 400)
        np.testing.assert_allclose(preprocess_emg(raw, 400, highcut_hz=150), 0.0)

    def test_short_and_nonfinite_recordings_are_rejected(self) -> None:
        for raw in (np.ones(27), np.array([1.0, np.nan]), np.ones((2, 40))):
            with self.subTest(shape=raw.shape), self.assertRaises(ValueError):
                preprocess_emg(raw, 4000)

    def test_rate_uses_device_timestamps_and_rejects_gaps_and_duplicates(self) -> None:
        self.assertEqual(sample_rate(np.arange(100) * 250), 4000)
        for times in ([0], [0, 250, 1000], [250, 0], [0, 0], [0, 1, 1], [0, np.nan]):
            with self.subTest(times=times), self.assertRaises(ValueError):
                sample_rate(np.asarray(times))

    def test_rms_and_mav_keep_full_windows_without_zero_padding(self) -> None:
        starts, rms, mav = window_features(np.full(501, -3.0), 1000)
        np.testing.assert_array_equal(starts, [0, 150, 300])
        np.testing.assert_allclose(rms, 3)
        np.testing.assert_allclose(mav, 3)
        for window, overlap in ((0, 0), (200, 200), (200, -1), (np.inf, 0)):
            with self.subTest(window=window, overlap=overlap), self.assertRaises(ValueError):
                window_features(np.ones(1000), 1000, window, overlap)

    def test_usb_and_ble_loaders_read_the_same_single_channel(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "recording.csv"
            path.write_text(
                "timestamp_us;emg_raw;imu1_ax\n0;2048;\n125;;1\n250;2049;\n", encoding="utf-8"
            )
            times, values = load_emg(path)
            np.testing.assert_array_equal(times, [0, 250])
            np.testing.assert_array_equal(values, [2048, 2049])
            path.write_text("device_time,channel,value\n0,0,2048\n250,0,2049\n", encoding="utf-8")
            ble_times, ble_values = load_emg(path)
            np.testing.assert_array_equal(ble_times, times)
            np.testing.assert_array_equal(ble_values, values)
            path.write_text("device_time,channel,value\n,0,2048\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "timestamp"):
                load_emg(path)

    def test_command_saves_processed_data_and_keeps_source_unchanged(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "emg.csv"
            source = "device_time,channel,value\n" + "".join(
                f"{index * 250},0,2048\n" for index in range(1000)
            )
            path.write_text(source, encoding="utf-8")
            with patch.object(sys, "argv", ["emg_processing.py", "--file", str(path)]):
                self.assertEqual(main(), 0)
            self.assertEqual(path.read_text(encoding="utf-8"), source)
            with path.with_suffix(".features.csv").open(newline="", encoding="utf-8") as output:
                features = list(csv.DictReader(output))
            self.assertEqual(len(features), 1)
            self.assertEqual(float(features[0]["rms"]), 0.0)
            metadata = json.loads(path.with_suffix(".processing.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["sample_rate_hz"], 4000)
            self.assertEqual(metadata["source"], "emg.csv")
            self.assertEqual(
                metadata["source_sha256"], hashlib.sha256(path.read_bytes()).hexdigest()
            )

    def test_labelled_processing_honours_filter_and_window_options(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            source = folder / "emg.csv"
            labels = folder / "emg_trial.json"
            origin = 1_000_000_000
            labels.write_text(
                json.dumps(
                    {
                        "start_host_monotonic_ns": origin,
                        "complete": False,
                        "segments": [{"trial": 1, "phase": "rest", "start_s": 0, "end_s": 2}],
                    }
                )
            )
            samples = 2048 + 20 * np.sin(2 * np.pi * 100 * np.arange(800) / 400)
            with source.open("w", newline="") as output:
                writer = csv.writer(output)
                writer.writerow(("device_time", "value", "synced_host_monotonic_ns"))
                writer.writerows(
                    (index * 2500, value, origin + index * 2_500_000)
                    for index, value in enumerate(samples)
                )
            original = source.read_bytes()
            with patch.object(
                sys,
                "argv",
                [
                    "emg_processing",
                    "--file",
                    str(source),
                    "--labels",
                    str(labels),
                    "--highcut",
                    "150",
                    "--window-ms",
                    "100",
                    "--overlap-ms",
                    "20",
                ],
            ):
                self.assertEqual(main(), 0)
            metadata = json.loads(source.with_suffix(".processing.json").read_text())
            self.assertEqual(metadata["processing"]["highcut_hz"], 150)
            self.assertEqual(metadata["processing"]["window_ms"], 100)
            self.assertEqual(metadata["processing"]["overlap_ms"], 20)
            self.assertEqual(metadata["sample_rate_hz"], 400)
            self.assertEqual(
                metadata["labels_sha256"], hashlib.sha256(labels.read_bytes()).hexdigest()
            )
            self.assertGreater(metadata["windows_by_phase"]["rest"], 15)
            self.assertEqual(source.read_bytes(), original)

    def test_processed_samples_and_features_can_be_plotted_without_hardware(self) -> None:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "processed.csv"
            image = Path(temporary) / "processed.png"
            for content, titles in (
                ("timestamp_us,emg_raw,emg_filtered\n0,2048,0\n250,2050,2\n", ["raw", "filtered"]),
                ("timestamp_us,rms,mav\n0,1,0.8\n150000,2,1.6\n", ["features"]),
            ):
                path.write_text(content, encoding="utf-8")
                with patch.object(
                    sys,
                    "argv",
                    ["plot_sensor_recording.py", "--file", str(path), "--save", str(image)],
                ):
                    self.assertEqual(plot_main(), 0)
                self.assertGreater(image.stat().st_size, 0)
                figure = make_figure(path)
                self.assertEqual(len(figure.axes), len(titles))
                for axis, title in zip(figure.axes, titles, strict=True):
                    self.assertIn(title, axis.get_title().lower())
                plt.close("all")


if __name__ == "__main__":
    unittest.main()
