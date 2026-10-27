# Majd
"""Build from a source archive and check an installed copy outside the checkout."""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(*arguments: str, cwd: Path = ROOT) -> None:
    subprocess.run([sys.executable, *arguments], cwd=cwd, check=True)


def extract_source_archive(archive: Path, destination: Path) -> None:
    destination = destination.resolve()
    with tarfile.open(archive) as source:
        for member in source.getmembers():
            if not (destination / member.name).resolve().is_relative_to(destination) or not (
                member.isfile() or member.isdir()
            ):
                raise ValueError(f"Unexpected source archive entry: {member.name!r}")
        source.extractall(destination)


def main() -> int:
    os.environ["PYTHONUTF8"] = "1"
    os.environ["MPLBACKEND"] = "Agg"
    with tempfile.TemporaryDirectory(prefix="aurora_package_") as temporary:
        # Temporary folders can use short Windows paths or symlinked macOS paths.
        work = Path(temporary).resolve()
        archive_directory = work / "source"
        archive_directory.mkdir()
        run(
            "-c",
            "import sys; from setuptools.build_meta import build_sdist; build_sdist(sys.argv[1])",
            str(archive_directory),
        )
        archive = next(archive_directory.glob("*.tar.gz"))
        extract_source_archive(archive, archive_directory)
        source_root = next(path for path in archive_directory.iterdir() if path.is_dir())
        wheels = work / "wheels"
        run(
            "-m",
            "pip",
            "wheel",
            ".",
            "--no-deps",
            "--no-build-isolation",
            "--wheel-dir",
            str(wheels),
            cwd=source_root,
        )
        installation = work / "installed copy \u00c5"
        wheel = next(wheels.glob("*.whl"))
        run(
            "-m",
            "pip",
            "install",
            "--no-deps",
            "--no-index",
            "--target",
            str(installation),
            str(wheel),
            cwd=work,
        )
        code = r"""
import hashlib, importlib, json, pathlib, re, runpy, sys, urllib.parse
sys.path.insert(0, sys.argv[1])
from aurora.paths import CHECKOUT_DIRECTORY, MODEL_DIRECTORY, PACKAGE_DIRECTORY, REPOSITORY_ROOT, RESOURCE_DIRECTORY
from aurora.gui.project_support import discover_recording_programs, discover_firmware_projects, DOCUMENTATION_FILE
assert CHECKOUT_DIRECTORY is None
assert PACKAGE_DIRECTORY.is_relative_to(pathlib.Path(sys.argv[1]))
from importlib.metadata import distribution
entry = next(item for item in distribution('aurora-arm').entry_points if item.name == 'aurora')
assert entry.load().__module__ == 'aurora.gui.app'
assert REPOSITORY_ROOT == pathlib.Path.cwd()
assert DOCUMENTATION_FILE.is_file()
assert (RESOURCE_DIRECTORY / 'README.md').is_file()
assert (RESOURCE_DIRECTORY / 'examples/simulation/demo/angles.npz').is_file()
assert len(discover_recording_programs()) >= 4
assert len(discover_firmware_projects()) == 1
from aurora.gui.tool_catalog import TOOLS
from aurora.gui.workflow_tools import WorkflowTools
package_arguments = list(sys.argv)
for tool in TOOLS.values():
    if not tool.script:
        continue
    script = WorkflowTools.script_path(tool.module)
    assert script.is_file(), script
    assert script.is_relative_to(RESOURCE_DIRECTORY), script
    sys.argv = [str(script), '--help']
    try:
        runpy.run_path(str(script), run_name='__main__')
    except SystemExit as error:
        assert error.code == 0, (script, error.code)
sys.argv = package_arguments
for name in ('aurora.ml.train_emg', 'aurora.ml.emg_model', 'aurora.ml.emg_network',
             'aurora.ml.emg_results', 'aurora.gui.emg_review',
             'aurora.recording.emg_labels', 'aurora.sensors.fusion_comparison'):
    module = importlib.import_module(name)
    assert pathlib.Path(module.__file__).is_relative_to(PACKAGE_DIRECTORY), name
expected = json.loads(sys.argv[2])
for name, digest in expected.items():
    assert hashlib.sha256((MODEL_DIRECTORY / name).read_bytes()).hexdigest() == digest
for document in PACKAGE_DIRECTORY.rglob('*.md'):
    for value in re.findall(r'\]\((<[^>]+>|[^\s)]+)\)', document.read_text(encoding='utf-8-sig')):
        parsed = urllib.parse.urlsplit(value.strip('<>'))
        if parsed.scheme or parsed.netloc or not parsed.path:
            continue
        assert (document.parent / urllib.parse.unquote(parsed.path)).exists(), (document, value)
sys.argv = ['aurora', '--help']
try:
    runpy.run_module('aurora.gui.app', run_name='__main__')
except SystemExit as error:
    assert error.code == 0, error.code
else:
    raise AssertionError('App --help must exit before opening a window')
sys.argv = ['trajectory_playback', '--headless']
runpy.run_module('aurora.simulation.trajectory_playback', run_name='__main__')
print('Installed package resources and headless simulation passed.')
"""
        hashes = {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (ROOT / "artifacts" / "models").glob("*.onnx")
        }
        run("-I", "-c", code, str(installation), json.dumps(hashes), cwd=work)
        signals = """
import hashlib, pathlib, sys
sys.path.insert(0, sys.argv[1])
import numpy as np
from aurora.recording.emg_processing import main as process
from aurora.recording.plot_sensor_recording import main as plot
path = pathlib.Path('emg.csv')
times = np.arange(4096) / 4000
values = 2048 + 100 * np.sin(2 * np.pi * 125 * times)
path.write_text('device_time,value\\n' + ''.join(f'{t*1e6},{v}\\n' for t,v in zip(times,values)))
original = hashlib.sha256(path.read_bytes()).digest()
sys.argv = ['emg_processing', '--file', str(path)]
assert process() == 0
assert plot(['--file', str(path), '--spectrum', '--save', 'raw.png']) == 0
assert plot(['--file', 'emg.filtered.csv', '--save', 'filtered.png']) == 0
assert hashlib.sha256(path.read_bytes()).digest() == original
from aurora.gui.tool_jobs import main as export
from aurora.gui.tool_catalog import arguments_for
from aurora.paths import RESOURCE_DIRECTORY
assert export(arguments_for('bundle', {'source': str(path), 'output': 'raw bundle.zip'})) == 0
assert export(arguments_for('dmp_export', {'source': str(RESOURCE_DIRECTORY / 'examples/simulation/demo'), 'output': 'DMP export'})) == 0
assert pathlib.Path('DMP export/trajectory.csv').is_file()
print('Installed raw plotting, spectrum and EMG processing passed.')
"""
        run("-I", "-c", signals, str(installation), cwd=work)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
