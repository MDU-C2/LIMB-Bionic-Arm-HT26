"""Lossless recording bundles and fitted DMP exports, separate from source data."""

from __future__ import annotations

import hashlib
import json
import zipfile
from pathlib import Path

from aurora.json_utils import write_json
from aurora.recording.emg_labels import digest
from aurora.time_utils import utc_now


def export_recording(source: Path, destination: Path, *, cancelled=None, progress=None):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if not source.exists():
        raise ValueError("Choose an existing recording file or trial folder.")
    if source.is_file() and source.name == "trial.json":
        source = source.parent
    if destination == source or (source.is_dir() and destination.is_relative_to(source)):
        raise ValueError("Save the bundle outside the original recording folder.")
    files = sorted(source.rglob("*")) if source.is_dir() else [source]
    files = [p for p in files if p.is_file()]
    if not files:
        raise ValueError("The selected recording contains no files.")
    if any(
        p.is_symlink()
        or not p.resolve().is_relative_to(source if source.is_dir() else source.parent)
        for p in files
    ):
        raise ValueError("Recording bundles cannot follow linked files outside the recording.")
    root = source if source.is_dir() else source.parent
    states = {p: (p.stat().st_size, p.stat().st_mtime_ns) for p in files}
    destination.parent.mkdir(parents=True, exist_ok=True)
    owned = False
    try:
        stream = destination.open("xb")
        owned = True
        records = []
        with stream, zipfile.ZipFile(stream, "w", compression=zipfile.ZIP_STORED) as archive:
            if progress:
                progress(0, len(files))
            for index, path in enumerate(files):
                if cancelled is not None and cancelled():
                    raise InterruptedError("Export cancelled; original recordings are unchanged.")
                checksum = hashlib.sha256()
                relative = path.relative_to(root).as_posix()
                with (
                    path.open("rb") as incoming,
                    archive.open("recording/" + relative, "w", force_zip64=True) as outgoing,
                ):
                    while block := incoming.read(1024 * 1024):
                        if cancelled is not None and cancelled():
                            raise InterruptedError(
                                "Export cancelled; original recordings are unchanged."
                            )
                        outgoing.write(block)
                        checksum.update(block)
                records.append(
                    dict(file=relative, bytes=states[path][0], sha256=checksum.hexdigest())
                )
                if progress:
                    progress(index + 1, len(files))
            if any(
                (p.stat().st_size, p.stat().st_mtime_ns) != state for p, state in states.items()
            ):
                raise ValueError("The recording changed during export. Stop recording and repeat.")
            archive.writestr(
                "AURORA-export.json",
                json.dumps(dict(format_version=1, created_utc=utc_now(), files=records), indent=2),
            )
        return destination
    except BaseException:
        if owned:
            destination.unlink(missing_ok=True)
        raise


def export_dmp(source: Path, output: Path, *, preferred="clean", refit=True):
    import numpy as np

    from aurora.simulation.dmp.trajectory_io import load_dmp_trajectory

    source, output = Path(source).resolve(), Path(output).resolve()
    if not source.is_dir() or preferred not in {"clean", "raw"}:
        raise ValueError("Choose a trajectory folder and clean or raw angles.")
    if output == source or output.is_relative_to(source):
        raise ValueError("Save the export outside the inherited trajectory folder.")
    inputs = {p.name: digest(p) for p in source.glob("*.npz")}
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(
        format_version=1,
        status="running",
        created_utc=utc_now(),
        source=str(source),
        source_sha256=inputs,
        units="rad",
        joint_order=[
            "elbow_flexion",
            "shoulder_flexion",
            "shoulder_abduction",
            "shoulder_internal_rotation",
        ],
        refit=refit,
        preferred_source=preferred,
        limits="Applied during robot playback; exported fitted samples are unclipped.",
    )
    write_json(output / "trajectory.json", manifest)
    try:
        trajectory, dt = load_dmp_trajectory(
            source, rollout_source=preferred, prefer_saved_rollout=not refit
        )
        target = output / f"dmp_rollout_{preferred}.npz"
        np.savez(target, q_gen_rad=trajectory, dt=dt)
        np.savetxt(
            output / "trajectory.csv",
            np.column_stack((np.arange(len(trajectory)) * dt, trajectory)),
            delimiter=",",
            header="time_s,elbow_flexion_rad,shoulder_flexion_rad,shoulder_abduction_rad,shoulder_internal_rotation_rad",
            comments="",
        )
        manifest.update(
            status="complete",
            samples=len(trajectory),
            dt_s=dt,
            artifacts={
                target.name: digest(target),
                "trajectory.csv": digest(output / "trajectory.csv"),
            },
        )
        return output
    except BaseException as error:
        manifest.update(
            status="interrupted"
            if isinstance(error, (KeyboardInterrupt, InterruptedError))
            else "failed",
            error=str(error),
        )
        raise
    finally:
        write_json(output / "trajectory.json", manifest)
