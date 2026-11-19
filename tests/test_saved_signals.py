# Majd
"""Check raw plotting, clock alignment and spectra without connected sensors."""

from __future__ import annotations

import csv
import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")

from aurora.recording.plot_sensor_recording import main, make_figure
from aurora.recording.saved_signals import Trace, emg_file, load_signals, with_gaps


def write_csv(path, columns, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(columns)
        writer.writerows(rows)


class SavedSignalTests(unittest.TestCase):
    def test_raw_trial_shares_real_host_times_and_preserves_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            origin = 10_000_000_000
            write_csv(
                folder / "ble/emg.csv",
                ["device_time", "synced_host_monotonic_ns", "sensor", "value"],
                [
                    [1_000_000, origin + 100_000_000, 0, 2048],
                    [1_000_250, origin + 100_250_000, 0, 2200],
                ],
            )
            columns = [
                "device_time",
                "synced_host_monotonic_ns",
                "sensor",
                "accel_x",
                "accel_y",
                "accel_z",
                "gyro_x",
                "gyro_y",
                "gyro_z",
            ]
            write_csv(
                folder / "ble/imu.csv",
                columns,
                [
                    [1_010_000, origin + 110_000_000, 0, 0, 1, 0, 1, 2, 3],
                    [1_020_000, origin + 120_000_000, 1, 1, 0, 0, 4, 5, 6],
                ],
            )
            (folder / "ble/meta.json").write_text(json.dumps({"imu_units": ["g", "dps"]}))
            (folder / "camera").mkdir()
            (folder / "camera/pose.json").write_text(
                json.dumps(
                    {
                        "observations": [
                            {
                                "host_monotonic_ns": origin + 130_000_000,
                                "angles_deg": {"elbow_flexion": 85},
                            }
                        ]
                    }
                )
            )
            trial = folder / "trial.json"
            trial.write_text(
                json.dumps(
                    {
                        "subject_id": "S01",
                        "trial_id": "T001",
                        "pace_condition": "slow",
                        "started_host_monotonic_ns": origin,
                        "operator_events": [{"marker": 2, "time_s": 0.2}],
                    }
                )
            )
            before = {
                p: hashlib.sha256(p.read_bytes()).hexdigest()
                for p in folder.rglob("*")
                if p.is_file()
            }
            data = load_signals(trial)
            self.assertEqual(len(data.panels), 6)
            self.assertEqual(data.title, "S01 / T001 / slow")
            self.assertEqual({p.clock for p in data.panels}, {"Time from trial start [s]"})
            self.assertAlmostEqual(data.panels[0].traces[0].time_s[0], 0.1)
            np.testing.assert_array_equal(data.panels[0].traces[0].values, [2048, 2200])
            self.assertEqual(data.panels[1].unit, "g")
            self.assertEqual(data.panels[2].unit, "dps")
            self.assertAlmostEqual(data.panels[-1].traces[0].time_s[0], 0.13)
            self.assertEqual(data.events, [(0.2, "Operator 2")])
            self.assertEqual(emg_file(trial), folder / "ble/emg.csv")
            self.assertEqual(
                main(["--file", str(trial), "--save", str(folder / "plots/raw.png")]), 0
            )
            for path, digest in before.items():
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)

    def test_usb_uses_each_imus_timestamp_and_keeps_axes_separate(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "recording.csv"
            path.write_text(
                "timestamp_us;emg_raw;imu1_timestamp_us;imu1_ax;imu1_gx;imu2_timestamp_us;imu2_ay;imu2_gy\n1000000;2000;1000100;500;10;1000200;700;20\n"
            )
            panels = load_signals(path).panels
            self.assertEqual(len(panels), 5)
            self.assertAlmostEqual(panels[1].traces[0].time_s[0], 0.0001)
            self.assertAlmostEqual(panels[3].traces[1].time_s[0], 0.0002)
            self.assertEqual(panels[1].unit, "mg")
            self.assertEqual(panels[2].unit, "mdps")

    def test_missing_sync_does_not_claim_camera_and_imu_alignment(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            write_csv(
                folder / "ble/emg.csv", ["device_time", "value"], [[1000, 2048], [1250, 2050]]
            )
            (folder / "camera").mkdir()
            (folder / "camera/pose.json").write_text(
                json.dumps({"observations": [{"time_s": 0.1, "angles_deg": {"elbow_flexion": 80}}]})
            )
            (folder / "trial.json").write_text(
                json.dumps({"started_host_monotonic_ns": 1_000_000_000})
            )
            panels = load_signals(folder).panels
            self.assertNotEqual(panels[0].clock, panels[1].clock)

    def test_plot_breaks_at_gaps_without_changing_values(self):
        trace = Trace("raw", np.array([0, 0.001, 0.002, 0.01]), np.array([10.0, 11.0, 12.0, 13.0]))
        times, values = with_gaps(trace)
        self.assertTrue(np.isnan(values[3]))
        np.testing.assert_array_equal(values[np.isfinite(values)], trace.values)
        self.assertEqual(len(times), 5)

    def test_welch_peak_matches_synthetic_frequency_and_rejects_gaps(self):
        import matplotlib.pyplot as plt

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "emg.csv"
            times = np.arange(4096) / 4000
            raw = 2048 + 100 * np.sin(2 * np.pi * 125 * times)
            write_csv(path, ["device_time", "value"], zip(times * 1e6, raw, strict=True))
            figure = make_figure(path, spectrum=True)
            line = figure.axes[-1].lines[0]
            self.assertAlmostEqual(line.get_xdata()[np.argmax(line.get_ydata())], 125, delta=1)
            plt.close(figure)
            write_csv(path, ["device_time", "value"], [[0, 2000], [250, 2200], [1250, 2300]])
            with self.assertRaisesRegex(ValueError, "regularly spaced"):
                make_figure(path, spectrum=True)

    def test_processed_features_are_not_accepted_as_raw_emg(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "emg.features.csv"
            path.write_text("timestamp_us,rms,mav\n0,1,0.8\n")
            with self.assertRaises(ValueError):
                emg_file(path)

    def test_spectrum_uses_device_sampling_despite_host_clock_corrections(self):
        import matplotlib.pyplot as plt

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "emg.csv"
            times = np.arange(4096) / 4000
            raw = 2048 + 100 * np.sin(2 * np.pi * 125 * times)
            host_ns = 10_000_000_000 + times * 1e9 + (np.arange(len(times)) // 40) * 100_000
            write_csv(
                path,
                ["device_time", "synced_host_monotonic_ns", "value"],
                zip(times * 1e6, host_ns, raw, strict=True),
            )
            trace = load_signals(path).panels[0].traces[0]
            self.assertFalse(np.allclose(np.diff(trace.time_s), 1 / 4000))
            figure = make_figure(path, spectrum=True)
            line = figure.axes[-1].lines[0]
            self.assertAlmostEqual(line.get_xdata()[np.argmax(line.get_ydata())], 125, delta=1)
            plt.close(figure)


if __name__ == "__main__":
    unittest.main()
