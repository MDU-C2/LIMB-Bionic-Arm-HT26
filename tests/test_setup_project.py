"""Check installer recovery and launch behavior without downloads or hardware."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import subprocess
import unittest
from unittest.mock import call, patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src/gui"))

import setup_project
from project_support import simulation_python_candidates


class SetupTests(unittest.TestCase):
    def test_dependency_conflicts_fail_verification_before_native_checks(self) -> None:
        with (
            patch.object(setup_project.subprocess, "run", return_value=subprocess.CompletedProcess(
                ["python", "-m", "pip", "check"], 1,
                "mediapipe requires protobuf<5\ndepthai 3.10.0 is not supported on this platform\n",
            )),
            patch.object(setup_project, "run_python") as checks,
        ):
            with self.assertRaises(subprocess.CalledProcessError):
                setup_project.verify()
            checks.assert_not_called()

    def test_camera_metadata_warning_still_requires_native_camera_import(self) -> None:
        with (
            patch.object(setup_project.subprocess, "run", return_value=subprocess.CompletedProcess(
                ["python", "-m", "pip", "check"], 1,
                "depthai 3.10.0 is not supported on this platform\n",
            )),
            patch.object(setup_project, "run_python") as checks,
        ):
            setup_project.verify()
            self.assertIn(call("-c", "import depthai"), checks.call_args_list)

    def test_ready_marker_is_invalidated_by_dependency_changes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dependency = root / "requirements.txt"
            dependency.write_text("numpy==1.26.4\n", encoding="utf-8")
            marker = root / "ready.json"
            with (
                patch.object(setup_project, "ROOT", root),
                patch.object(setup_project, "INPUTS", ("requirements.txt",)),
                patch.object(setup_project, "READY", marker),
            ):
                self.assertFalse(setup_project.is_ready())
                marker.write_text(json.dumps({"fingerprint": setup_project.fingerprint()}))
                self.assertTrue(setup_project.is_ready())
                dependency.write_text("numpy==2.0.0\n", encoding="utf-8")
                self.assertFalse(setup_project.is_ready())

    def test_partial_or_corrupt_marker_requires_setup(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            marker = Path(temporary) / "ready.json"
            with patch.object(setup_project, "READY", marker):
                for contents in ("", "{", "null", "[]", "{}"):
                    marker.write_text(contents, encoding="utf-8")
                    self.assertFalse(setup_project.is_ready())

    def test_fresh_launch_requests_setup_without_starting_app(self) -> None:
        with (
            patch.object(setup_project, "is_ready", return_value=False),
            patch.object(setup_project.subprocess, "call") as launch,
            patch.dict(os.environ),
        ):
            self.assertEqual(setup_project.main(["--launch"]), setup_project.NEEDS_SETUP)
            launch.assert_not_called()

    def test_ready_launch_preserves_paths_and_app_exit_code(self) -> None:
        root = ROOT / "clone with spaces and Å"
        with (
            patch.object(setup_project, "ROOT", root),
            patch.object(setup_project, "is_ready", return_value=True),
            patch.object(setup_project.subprocess, "call", return_value=7) as launch,
            patch.dict(os.environ),
        ):
            self.assertEqual(setup_project.main(["--launch"]), 7)
            launch.assert_called_once_with(
                [sys.executable, str(root / "src/gui/app.py")], cwd=root,
            )
            self.assertEqual(os.environ["AURORA_SIMULATION_PYTHON"], sys.executable)

    def test_failed_firmware_build_never_marks_setup_complete(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            state = Path(temporary)
            marker = state / "ready.json"
            marker.write_text('{"fingerprint": "old"}')
            with (
                patch.object(setup_project, "STATE", state),
                patch.object(setup_project, "READY", marker),
                patch.object(setup_project, "verify"),
                patch.object(setup_project, "run_python") as run,
            ):
                def fail_build(*arguments, **kwargs):
                    if arguments[0] == "src/gui/firmware_workspace.py":
                        raise OSError("Toolchain download interrupted")

                run.side_effect = fail_build
                with self.assertRaises(OSError):
                    setup_project.install()
                self.assertFalse(marker.exists())

    def test_gui_discovers_windows_cached_environment_before_system_python(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            state = root / ".aurora"
            state.mkdir()
            prefix = root / "cache"
            (state / "runtime.json").write_text(
                json.dumps({"prefix": str(prefix)}), encoding="utf-8-sig",
            )
            with (
                patch("project_support.REPOSITORY_ROOT", root),
                patch.dict(os.environ, {"AURORA_SIMULATION_PYTHON": ""}),
            ):
                python = "python.exe" if os.name == "nt" else "bin/python"
                self.assertEqual(simulation_python_candidates()[0], (prefix / python).resolve())


if __name__ == "__main__":
    unittest.main()
