# Majd
"""Fit sensor-to-arm rotations from isolated movements and a stereo reference.

Segment X is the positive flexion axis, Y points along the arm towards the hand,
and Z = X cross Y. Matrices map sensor vectors into these segment coordinates.
The reference assumes an upright subject with the upper arm beside the body.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

METHOD = "isolated_axes_and_static_gravity_v1"
ROLES = ("shoulder", "wrist")


@dataclass(frozen=True)
class AlignmentSettings:
    # Collection checks to evaluate in the pilot, not published accuracy limits.
    min_samples: int = 100
    min_duration_s: float = 2.0
    max_sample_gap_s: float = 0.05
    max_still_gyro_dps: float = 5.0
    max_still_accel_spread_g: float = 0.05
    min_active_speed_dps: float = 10.0
    min_axis_fraction: float = 0.85
    min_excursion_deg: float = 25.0
    min_travel_deg: float = 60.0
    max_axis_error_deg: float = 15.0
    max_return_error_deg: float = 12.0
    min_camera_frames: int = 5


@dataclass(frozen=True)
class ImuSeries:
    time_s: np.ndarray
    accel_g: np.ndarray
    gyro_dps: np.ndarray


class AlignmentError(ValueError):
    """Name the preparation step which needs repeating."""


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if vector.shape != (3,) or not np.isfinite(vector).all() or length < 1e-8:
        raise AlignmentError("A reference direction is missing or degenerate")
    return vector / length


def angle_deg(first, second):
    return float(np.degrees(np.arccos(np.clip(np.dot(unit(first), unit(second)), -1, 1))))


def rotation_matrix(value):
    """Reject reflections and malformed profiles before rotating saved samples."""
    matrix = np.asarray(value, dtype=float)
    if (
        matrix.shape != (3, 3)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6)
    ):
        raise ValueError("Sensor-to-arm matrix must be a finite proper rotation")
    return matrix


def _series(stages, stage, role, settings):
    label = f"{stage} {'upper arm' if role == 'shoulder' else 'forearm'}"
    try:
        values = stages[stage][role]
    except KeyError as error:
        raise AlignmentError(f"{label}: no IMU samples") from error
    times, accel, gyro = values.time_s, values.accel_g, values.gyro_dps
    if (
        len(times) < settings.min_samples
        or accel.shape != (len(times), 3)
        or gyro.shape != (len(times), 3)
        or not all(np.isfinite(item).all() for item in (times, accel, gyro))
    ):
        raise AlignmentError(f"{label}: too few valid IMU samples")
    intervals = np.diff(times)
    if np.any(intervals <= 0) or np.max(intervals) > settings.max_sample_gap_s:
        raise AlignmentError(f"{label}: IMU timing has gaps; check BLE and repeat")
    if times[-1] - times[0] < settings.min_duration_s:
        raise AlignmentError(f"{label}: recording is too short")
    return values


def _still(series, label, settings):
    gravity = np.mean(series.accel_g, axis=0)
    bias = np.mean(series.gyro_dps, axis=0)
    spread = float(np.sqrt(np.mean(np.sum((series.accel_g - gravity) ** 2, axis=1))))
    speed = float(np.percentile(np.linalg.norm(series.gyro_dps, axis=1), 95))
    if not 0.85 <= np.linalg.norm(gravity) <= 1.15:
        raise AlignmentError(f"{label}: acceleration is not a stationary 1 g reference")
    if speed > settings.max_still_gyro_dps or spread > settings.max_still_accel_spread_g:
        raise AlignmentError(f"{label}: subject must remain still")
    return gravity, bias, {"gyro_p95_dps": speed, "accel_spread_g": spread}


def _axis(series, bias, label, settings):
    """For an isolated rotation, gyro ~= rate * axis.

    The leading eigenvector of sum(gyro * gyro.T) estimates that axis even
    when the subject reverses direction. Its sign needs the first-motion cue.
    """
    rates = series.gyro_dps - bias
    active = rates[np.linalg.norm(rates, axis=1) >= settings.min_active_speed_dps]
    if len(active) < 30:
        raise AlignmentError(f"{label}: insufficient movement")
    eigenvalues, eigenvectors = np.linalg.eigh(active.T @ active)
    fraction = float(eigenvalues[-1] / np.sum(eigenvalues))
    if fraction < settings.min_axis_fraction:
        raise AlignmentError(f"{label}: mixed movement; follow the isolated movement cue")
    axis = eigenvectors[:, -1]
    dt = np.diff(series.time_s, prepend=series.time_s[0])
    displacement = np.cumsum((rates @ axis) * dt)
    first_motion = np.flatnonzero(np.abs(displacement) >= 5)
    if not len(first_motion):
        raise AlignmentError(f"{label}: no clear first movement")
    # The cues specify raising/bending first. Stereo checks that direction too.
    if displacement[first_motion[0]] < 0:
        axis, displacement = -axis, -displacement
    excursion = float(np.percentile(displacement, 97.5) - np.percentile(displacement, 2.5))
    travel = float(np.sum(np.abs(rates @ axis) * dt))
    if excursion < settings.min_excursion_deg or travel < settings.min_travel_deg:
        raise AlignmentError(f"{label}: movement range is too small; repeat comfortably")
    return axis, {"axis_fraction": fraction, "excursion_deg": excursion, "travel_deg": travel}


def _basis(x_axis, y_axis):
    x_axis = unit(x_axis)
    y_axis = unit(y_axis - np.dot(x_axis, y_axis) * x_axis)
    matrix = np.row_stack((x_axis, y_axis, np.cross(x_axis, y_axis)))
    return rotation_matrix(matrix)


def _camera_points(frames, side, stage, settings):
    required = ("left_shoulder", "right_shoulder", f"{side}_elbow", f"{side}_wrist")
    points = []
    for frame in frames.get(stage, []):
        try:
            value = {key: np.asarray(frame[key], dtype=float) for key in required}
            if any(p.shape != (3,) or not np.isfinite(p).all() for p in value.values()):
                continue
            upper = value[f"{side}_elbow"] - value[f"{side}_shoulder"]
            forearm = value[f"{side}_wrist"] - value[f"{side}_elbow"]
            if not (
                0.12 <= np.linalg.norm(upper) <= 0.6 and 0.12 <= np.linalg.norm(forearm) <= 0.6
            ):
                continue
            points.append((value, unit(upper), unit(forearm)))
        except (KeyError, TypeError, ValueError):
            continue
    if len(points) < settings.min_camera_frames:
        raise AlignmentError(f"{stage}: insufficient stereo arm points; check view and depth")
    return points


def fit_alignment(stages, camera_frames, *, side, settings=None):
    """Return rotations and collection checks without altering original samples.

    This is a constrained functional calibration, not the three-sensor/magnetic
    optimisation in McInnes et al. The research basis and assumptions are documented
    in research/related-work.md.
    """
    settings = settings or AlignmentSettings()
    if side not in ("left", "right"):
        raise ValueError("Choose left or right arm")
    gravity, bias, metrics = {}, {}, {}
    for role in ROLES:
        still = _series(stages, "STILL", role, settings)
        gravity[role], bias[role], metrics[f"still_{role}"] = _still(still, "STILL", settings)
    upper_x, metrics["raise_upper_arm"] = _axis(
        _series(stages, "RAISE", "shoulder", settings), bias["shoulder"], "RAISE", settings
    )
    raise_forearm_x, metrics["raise_forearm"] = _axis(
        _series(stages, "RAISE", "wrist", settings), bias["wrist"], "RAISE", settings
    )
    forearm_x, metrics["bend_forearm"] = _axis(
        _series(stages, "BEND", "wrist", settings), bias["wrist"], "BEND", settings
    )
    forearm_y, metrics["turn_forearm"] = _axis(
        _series(stages, "TURN", "wrist", settings), bias["wrist"], "TURN", settings
    )
    for stage in ("BEND", "TURN"):
        values = _series(stages, stage, "shoulder", settings)
        speed = float(np.percentile(np.linalg.norm(values.gyro_dps - bias["shoulder"], axis=1), 95))
        metrics[f"{stage.lower()}_upper_arm_p95_dps"] = speed
        if speed > settings.max_still_gyro_dps:
            raise AlignmentError(f"{stage}: keep the upper arm still beside the body")
    checks = {
        "upper_axis_gravity_error_deg": abs(90 - angle_deg(upper_x, gravity["shoulder"])),
        "forearm_axis_orthogonality_error_deg": abs(90 - angle_deg(forearm_x, forearm_y)),
        "raise_bend_axis_error_deg": angle_deg(raise_forearm_x, forearm_x),
    }
    if any(value > settings.max_axis_error_deg for value in checks.values()):
        raise AlignmentError("RAISE/BEND/TURN axes disagree; keep thumb up and follow each cue")
    upper_matrix = _basis(upper_x, -unit(gravity["shoulder"]))
    forearm_matrix = _basis(forearm_x, forearm_y)
    if (forearm_matrix @ gravity["wrist"])[2] < 0:
        forearm_matrix = _basis(forearm_x, -forearm_y)

    neutral = _camera_points(camera_frames, side, "STILL", settings)
    upper_y_c = unit(np.median([p[1] for p in neutral], axis=0))
    forearm_y_c = unit(np.median([p[2] for p in neutral], axis=0))
    flexion = angle_deg(upper_y_c, forearm_y_c)
    if not 70 <= flexion <= 110:
        raise AlignmentError("STILL: elbow must be about 90 degrees")
    x_camera = unit(np.cross(upper_y_c, forearm_y_c))
    lateral = unit(
        np.median([p[0]["left_shoulder"] - p[0]["right_shoulder"] for p in neutral], axis=0)
    )
    if angle_deg(x_camera, -lateral) > settings.max_axis_error_deg:
        raise AlignmentError("STILL: point the forearm forward, with the upper arm beside the body")
    camera_from_segment = {
        "shoulder": _basis(x_camera, upper_y_c).T,
        "wrist": _basis(x_camera, forearm_y_c).T,
    }
    matrices = {"shoulder": upper_matrix, "wrist": forearm_matrix}
    for role in ROLES:
        predicted_up = camera_from_segment[role] @ matrices[role] @ unit(gravity[role])
        checks[f"{role}_gravity_reference_error_deg"] = angle_deg(predicted_up, -upper_y_c)
        if checks[f"{role}_gravity_reference_error_deg"] > settings.max_axis_error_deg:
            raise AlignmentError(
                "STILL/TURN: reference pose disagrees with IMU axes; keep thumb up"
            )
        end = _series(stages, "RETURN", role, settings)
        end_gravity, _, metrics[f"return_{role}"] = _still(end, "RETURN", settings)
        checks[f"{role}_return_error_deg"] = angle_deg(end_gravity, gravity[role])
        if checks[f"{role}_return_error_deg"] > settings.max_return_error_deg:
            raise AlignmentError("RETURN: restore the starting pose; check for a loose sensor")

    for stage in ("RAISE", "BEND", "RETURN"):
        frames = _camera_points(camera_frames, side, stage, settings)
        if stage == "RAISE":
            movement = [
                float(
                    np.degrees(
                        np.arctan2(
                            np.dot(x_camera, np.cross(upper_y_c, p[1])), np.dot(upper_y_c, p[1])
                        )
                    )
                )
                for p in frames
            ]
            elbow_errors = [abs(angle_deg(p[1], p[2]) - flexion) for p in frames]
            if np.percentile(elbow_errors, 90) > settings.max_axis_error_deg:
                raise AlignmentError("RAISE: keep the elbow bent at the same angle")
        elif stage == "BEND":
            movement = [angle_deg(p[1], p[2]) - flexion for p in frames]
            if (
                np.percentile([angle_deg(p[1], upper_y_c) for p in frames], 90)
                > settings.max_axis_error_deg
            ):
                raise AlignmentError("BEND: keep the upper arm beside the body")
        else:
            if (
                max(
                    angle_deg(unit(np.median([p[j] for p in frames], axis=0)), reference)
                    for j, reference in ((1, upper_y_c), (2, forearm_y_c))
                )
                > settings.max_return_error_deg
            ):
                raise AlignmentError("RETURN: restore the original arm position")
            continue
        first = next((value for value in movement if abs(value) >= 5), None)
        if first is None or first < 0 or max(movement) < 15:
            raise AlignmentError(
                f"{stage}: begin by raising/bending, then return; check stereo view"
            )
        metrics[f"camera_{stage.lower()}_range_deg"] = float(max(movement) - min(movement))

    return {
        "format_version": 1,
        "method": METHOD,
        "status": "passed",
        "side": side,
        "segment_frame": "x_positive_flexion_y_distal_z_cross_xy",
        "matrix_direction": "segment_from_sensor",
        "sensor_to_segment": {role: value.tolist() for role, value in matrices.items()},
        "gyro_bias_dps": {role: value.tolist() for role, value in bias.items()},
        "camera_from_segment_at_reference": {
            role: value.tolist() for role, value in camera_from_segment.items()
        },
        "reference_elbow_flexion_deg": flexion,
        "body_reference": "static_upright_upper_arm_and_stereo_shoulders",
        "settings": asdict(settings),
        "checks": checks,
        "metrics": metrics,
        "accuracy_validated": False,
    }
