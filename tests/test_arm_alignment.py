# Majd
"""Recover known mounting rotations from generated rigid-arm movements."""

from __future__ import annotations

import copy
import csv
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from aurora.recording.cup_protocol import REFERENCE_STEPS
from aurora.recording.subject_calibration import (
    export_aligned_imu,
    fit_subject_reference,
    reference_windows,
)
from aurora.sensors.arm_alignment import AlignmentError, ImuSeries, fit_alignment, rotation_matrix


def generated_reference(side="left"):
    """Sensors have arbitrary mountings; trajectories supply an independent truth."""
    mounting = {
        "shoulder": Rotation.from_euler("xyz", [23, -48, 71], degrees=True).as_matrix(),
        "wrist": Rotation.from_euler("xyz", [-67, 36, -112], degrees=True).as_matrix(),
    }
    bias = {"shoulder": np.array([0.3, -0.4, 0.2]), "wrist": np.array([-0.2, 0.1, 0.4])}
    upper_zero = np.diag([-1.0, -1.0, 1.0])
    forearm_zero = np.array([[-1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])
    stages, frames = {}, {}
    for name in ("STILL", "RAISE", "BEND", "TURN", "RETURN"):
        times = np.arange(0, 5 if name in ("STILL", "RETURN") else 10, 0.01)
        angle = np.radians(45) * (1 - np.cos(2 * np.pi * times / 5)) / 2
        rate = np.degrees(np.radians(45) * np.pi / 5 * np.sin(2 * np.pi * times / 5))
        upper = np.repeat(upper_zero[None], len(times), axis=0)
        forearm = np.repeat(forearm_zero[None], len(times), axis=0)
        upper_rates = np.zeros((len(times), 3))
        forearm_rates = np.zeros_like(upper_rates)
        if name == "RAISE":
            upper = upper_zero @ Rotation.from_rotvec(angle[:, None] * [1, 0, 0]).as_matrix()
            forearm = (upper @ upper_zero.T) @ forearm_zero
            upper_rates[:, 0] = forearm_rates[:, 0] = rate
        elif name == "BEND":
            forearm = forearm_zero @ Rotation.from_rotvec(angle[:, None] * [1, 0, 0]).as_matrix()
            forearm_rates[:, 0] = rate
        elif name == "TURN":
            turn_sign = 1 if side == "left" else -1
            angle = turn_sign * np.radians(80) * np.sin(2 * np.pi * times / 4)
            forearm = forearm_zero @ Rotation.from_rotvec(angle[:, None] * [0, 1, 0]).as_matrix()
            forearm_rates[:, 1] = turn_sign * np.degrees(
                np.radians(80) * np.pi / 2 * np.cos(2 * np.pi * times / 4)
            )
        stages[name] = {}
        for role, orientation, rates in (
            ("shoulder", upper, upper_rates),
            ("wrist", forearm, forearm_rates),
        ):
            gravity_segment = np.einsum("nji,j->ni", orientation, [0, 1, 0])
            stages[name][role] = ImuSeries(
                times, gravity_segment @ mounting[role], rates @ mounting[role] + bias[role]
            )
        frames[name] = []
        for index in range(0, len(times), 20):
            points = {
                "left_shoulder": np.array([0.2, 0, -1.6]),
                "right_shoulder": np.array([-0.2, 0, -1.6]),
            }
            points[f"{side}_elbow"] = points[f"{side}_shoulder"] + 0.3 * upper[index, :, 1]
            points[f"{side}_wrist"] = points[f"{side}_elbow"] + 0.26 * forearm[index, :, 1]
            frames[name].append(points)
    return stages, frames, mounting, bias


class ArmAlignmentTests(unittest.TestCase):
    def test_alignment_windows_exclude_reading_pauses_and_step_countdowns(self):
        manifest = {
            "protocol": {"reference_steps": [{"cue": "STILL"}, {"cue": "BEND"}]},
            "cue_events": [
                {
                    "phase": phase,
                    "reference_step": step,
                    "host_monotonic_ns": seconds * 1_000_000_000,
                }
                for phase, step, seconds in (
                    ("reference_prepare", 0, 1),
                    ("reference_countdown", 0, 10),
                    ("reference", 0, 13),
                    ("reference", 0, 14),
                    ("reference_prepare", 1, 18),
                    ("reference_countdown", 1, 60),
                    ("reference", 1, 63),
                )
            ],
            "recording_ended_host_monotonic_ns": 73_000_000_000,
        }
        windows = reference_windows(manifest)
        self.assertEqual(windows["STILL"], (14_000_000_000, 17_800_000_000))
        self.assertEqual(windows["BEND"], (63_250_000_000, 72_800_000_000))

    def test_mixed_rotation_axes_are_rejected(self):
        stages, camera, _, _ = generated_reference()
        values = stages["RAISE"]["shoulder"]
        values.gyro_dps[:, 2] += 60 * np.sin(2 * np.pi * values.time_s / 2)
        with self.assertRaisesRegex(AlignmentError, "mixed movement"):
            fit_alignment(stages, camera, side="left")

    def test_fast_small_turns_do_not_pass_the_movement_range_check(self):
        stages, camera, mounting, bias = generated_reference()
        values = stages["TURN"]["wrist"]
        rates = np.column_stack(
            (
                np.zeros(len(values.time_s)),
                100 * np.cos(2 * np.pi * values.time_s * 2),
                np.zeros(len(values.time_s)),
            )
        )
        stages["TURN"]["wrist"] = ImuSeries(
            values.time_s, values.accel_g, rates @ mounting["wrist"] + bias["wrist"]
        )
        with self.assertRaisesRegex(AlignmentError, "range is too small"):
            fit_alignment(stages, camera, side="left")

    def test_saved_reference_uses_actual_cues_and_keeps_source_files_unchanged(self):
        stages, frames, mounting, _ = generated_reference()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "ble").mkdir()
            (root / "camera").mkdir()
            events, poses, rows = [], [], []
            start_ns = 10_000_000_000
            for step, (name, duration, _instruction) in enumerate(REFERENCE_STEPS):
                events.append(
                    {"phase": "reference", "reference_step": step, "host_monotonic_ns": start_ns}
                )
                if name in stages:
                    for sensor, role in enumerate(("shoulder", "wrist")):
                        values = stages[name][role]
                        for index, stamp in enumerate(values.time_s):
                            host = start_ns + round(stamp * 1e9)
                            rows.append(
                                [
                                    host,
                                    host,
                                    host // 1000,
                                    sensor,
                                    *values.accel_g[index],
                                    *values.gyro_dps[index],
                                ]
                            )
                    for index, points in enumerate(frames[name]):
                        poses.append(
                            {
                                "host_monotonic_ns": start_ns + index * 200_000_000,
                                "keypoints_camera_m": {
                                    key: value.tolist() for key, value in points.items()
                                },
                            }
                        )
                start_ns += duration * 1_000_000_000
            path = root / "ble/imu.csv"
            with path.open("w", encoding="utf-8", newline="") as output:
                writer = csv.writer(output)
                writer.writerow(
                    [
                        "host_monotonic_ns",
                        "synced_host_monotonic_ns",
                        "device_time",
                        "sensor",
                        *[f"{kind}_{axis}" for kind in ("accel", "gyro") for axis in "xyz"],
                    ]
                )
                writer.writerows(sorted(rows, key=lambda row: row[0]))
            (root / "camera/pose.json").write_text(json.dumps({"data": poses}), encoding="utf-8")
            manifest = {
                "subject_id": "S01",
                "side": "left",
                "connection_generation": 0,
                "complete": True,
                "stereo_depth_enabled": True,
                "camera_intrinsics": [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
                "imu_units": ["g", "dps"],
                "protocol": {"reference_steps": [{"cue": cue} for cue, _, _ in REFERENCE_STEPS]},
                "cue_events": events,
                "recording_ended_host_monotonic_ns": start_ns,
            }
            before = path.read_bytes()
            result = fit_subject_reference(root, manifest)
            self.assertEqual(result["status"], "passed", result.get("message"))
            self.assertEqual(result["cue_assignment_clock"], "synced_host_monotonic_ns")
            self.assertEqual(before, path.read_bytes())
            for role in mounting:
                np.testing.assert_allclose(
                    result["sensor_to_segment"][role], mounting[role], atol=1e-10
                )
            manifest["complete"] = False
            self.assertEqual(fit_subject_reference(root, manifest)["status"], "failed")

    def test_arbitrary_sensor_mountings_recover_proper_rotations_for_both_arms(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                stages, camera, mounting, bias = generated_reference(side)
                profile = fit_alignment(stages, camera, side=side)
                self.assertEqual(profile["status"], "passed")
                self.assertFalse(profile["accuracy_validated"])
                for role in mounting:
                    matrix = rotation_matrix(profile["sensor_to_segment"][role])
                    np.testing.assert_allclose(matrix, mounting[role], atol=1e-10)
                    np.testing.assert_allclose(
                        profile["gyro_bias_dps"][role], bias[role], atol=1e-10
                    )
                    self.assertAlmostEqual(np.linalg.det(matrix), 1)

    def test_no_movement_cannot_produce_a_passed_alignment(self):
        stages, camera, _, _ = generated_reference()
        stages["RAISE"]["shoulder"].gyro_dps[:] = [0.3, -0.4, 0.2]
        with self.assertRaisesRegex(AlignmentError, "RAISE: insufficient movement"):
            fit_alignment(stages, camera, side="left")

    def test_moving_the_upper_arm_during_elbow_bends_is_rejected(self):
        stages, camera, _, _ = generated_reference()
        stages["BEND"]["shoulder"].gyro_dps[:, 0] += 20
        with self.assertRaisesRegex(AlignmentError, "BEND: keep the upper arm still"):
            fit_alignment(stages, camera, side="left")

    def test_packet_gap_is_not_interpolated_into_a_calibration(self):
        stages, camera, _, _ = generated_reference()
        stages["TURN"]["wrist"].time_s[500:] += 0.2
        with self.assertRaisesRegex(AlignmentError, "timing has gaps"):
            fit_alignment(stages, camera, side="left")

    def test_hidden_shoulders_and_unmeasured_depth_cannot_supply_the_reference(self):
        stages, camera, _, _ = generated_reference()
        for frame in camera["STILL"]:
            del frame["right_shoulder"]
        with self.assertRaisesRegex(AlignmentError, "STILL: insufficient stereo"):
            fit_alignment(stages, camera, side="left")

    def test_a_changed_return_pose_is_rejected(self):
        stages, camera, _, _ = generated_reference()
        original = stages["RETURN"]["wrist"]
        changed = original.accel_g @ Rotation.from_euler("x", 35, degrees=True).as_matrix()
        stages["RETURN"]["wrist"] = ImuSeries(original.time_s, changed, original.gyro_dps)
        with self.assertRaisesRegex(AlignmentError, "RETURN: restore"):
            fit_alignment(stages, camera, side="left")

    def test_reflections_cannot_be_used_as_mounting_rotations(self):
        with self.assertRaisesRegex(ValueError, "proper rotation"):
            rotation_matrix(np.diag([-1, 1, 1]))

    def test_aligned_export_preserves_raw_data_and_packet_timestamps(self):
        stages, camera, mounting, bias = generated_reference()
        profile = fit_alignment(stages, camera, side="left")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, destination = root / "imu.csv", root / "imu_aligned.csv"
            fields = [
                "host_monotonic_ns",
                "device_time",
                "sensor",
                *[f"{kind}_{axis}" for kind in ("accel", "gyro") for axis in "xyz"],
            ]
            with source.open("w", encoding="utf-8", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=fields)
                writer.writeheader()
                for sensor, role in enumerate(("shoulder", "wrist")):
                    raw_accel = np.array([0.2, -0.3, 0.8]) @ mounting[role]
                    raw_gyro = np.array([15, -10, 4]) @ mounting[role] + bias[role]
                    writer.writerow(
                        dict(
                            zip(
                                fields,
                                [123456789123, 9000000, sensor, *raw_accel, *raw_gyro],
                                strict=True,
                            )
                        )
                    )
            original = hashlib.sha256(source.read_bytes()).digest()
            result = export_aligned_imu(source, destination, profile, units=("g", "dps"))
            self.assertEqual(result["rows"], 2)
            self.assertEqual(hashlib.sha256(source.read_bytes()).digest(), original)
            with destination.open(encoding="utf-8") as output:
                rows = list(csv.DictReader(output))
            for row in rows:
                self.assertEqual(row["host_monotonic_ns"], "123456789123")
                self.assertEqual(row["device_time"], "9000000")
                np.testing.assert_allclose(
                    [float(row[f"accel_{axis}"]) for axis in "xyz"], [0.2, -0.3, 0.8], atol=1e-10
                )
                np.testing.assert_allclose(
                    [float(row[f"gyro_{axis}"]) for axis in "xyz"], [15, -10, 4], atol=1e-10
                )
            with self.assertRaises(FileExistsError):
                export_aligned_imu(source, destination, profile, units=("g", "dps"))
            bad = copy.deepcopy(profile)
            bad["sensor_to_segment"]["wrist"] = np.diag([1, 1, -1]).tolist()
            with self.assertRaisesRegex(ValueError, "proper rotation"):
                export_aligned_imu(source, root / "bad.csv", bad, units=("g", "dps"))


if __name__ == "__main__":
    unittest.main()
