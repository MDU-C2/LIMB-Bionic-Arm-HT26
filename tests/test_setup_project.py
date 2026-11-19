# Majd
"""Check installer recovery and launch behavior without downloads or hardware."""

from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, call, patch

ROOT = Path(__file__).resolve().parents[1]

import aurora.gui.firmware_workspace as firmware_workspace
import scripts.setup_project as setup_project
from aurora.gui.project_support import simulation_python_candidates


class SetupTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "nt", "Windows launcher")
    def test_start_uses_the_saved_python_without_installation_or_ready_marker(self) -> None:
        with tempfile.TemporaryDirectory(prefix="aurora launch ") as temporary:
            root = Path(temporary)
            (root / "scripts").mkdir()
            (root / ".aurora").mkdir()
            shutil.copy2(ROOT / "scripts/setup.ps1", root / "scripts/setup.ps1")
            (root / ".aurora/runtime.json").write_text(
                json.dumps({"prefix": str(Path(sys.executable).parent)}), encoding="utf-8"
            )
            (root / "scripts/setup_project.py").write_text(
                "import pathlib, sys\n"
                "assert sys.argv[1:] == ['--launch']\n"
                "pathlib.Path('started.txt').write_text(sys.executable)\n"
                "raise SystemExit(7)\n",
                encoding="utf-8",
            )
            result = subprocess.run(
                [
                    "powershell.exe",
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-File",
                    str(root / "scripts/setup.ps1"),
                    "-Start",
                ],
                cwd=root,
                env={**os.environ, "AURORA_RUNTIME_ROOT": "invalid setup path"},
                capture_output=True,
                text=True,
                timeout=20,
            )
            self.assertEqual(result.returncode, 7, result.stdout + result.stderr)
            self.assertEqual(Path((root / "started.txt").read_text()), Path(sys.executable))
            self.assertNotIn("Installing", result.stdout)

    def test_firmware_uses_the_runtime_when_the_system_temp_path_has_spaces(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "checkout with spaces"
            source.mkdir()
            (source / "CMakeLists.txt").write_text("project(sensor)\n", encoding="utf-8")
            runtime = root / "runtime"
            with (
                patch.dict(os.environ, {"AURORA_SETUP_DIRECTORY": str(runtime)}),
                patch.object(
                    firmware_workspace.tempfile, "gettempdir", return_value=str(root / "user name")
                ),
            ):
                staged = firmware_workspace.staged_project(source)
            self.assertTrue(staged.is_relative_to(runtime.resolve()))
            self.assertEqual((staged / "CMakeLists.txt").read_text(), "project(sensor)\n")

    def test_firmware_staging_recovers_after_an_interrupted_copy(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            (source / "main").mkdir(parents=True)
            (source / "CMakeLists.txt").write_text("project(sensor)\n", encoding="utf-8")
            original = source / "main" / "main.c"
            original.write_text("void app_main(void) {}\n", encoding="utf-8")
            with patch.dict(os.environ, {"AURORA_SETUP_DIRECTORY": str(root / "runtime")}):
                with (
                    patch.object(
                        firmware_workspace.shutil, "copy2", side_effect=OSError("Interrupted")
                    ),
                    self.assertRaises(OSError),
                ):
                    firmware_workspace.staged_project(source)
                staged = firmware_workspace.staged_project(source)
                self.assertEqual((staged / "main" / "main.c").read_bytes(), original.read_bytes())
                (staged / "main" / "main.c").write_text("partial", encoding="utf-8")
                self.assertEqual(firmware_workspace.staged_project(source), staged)
                self.assertEqual((staged / "main" / "main.c").read_bytes(), original.read_bytes())

    def test_direct_gui_launch_uses_the_saved_runtime_for_firmware(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / ".aurora").mkdir()
            runtime = root / "runtime"
            (root / ".aurora" / "runtime.json").write_text(
                json.dumps({"prefix": str(runtime / "env")}), encoding="utf-8-sig"
            )
            source = root / "firmware"
            source.mkdir()
            (source / "CMakeLists.txt").write_text("project(sensor)\n", encoding="utf-8")
            with (
                patch.dict(os.environ, {"AURORA_SETUP_DIRECTORY": ""}),
                patch.object(
                    firmware_workspace,
                    "__file__",
                    str(root / "src/aurora/gui/firmware_workspace.py"),
                ),
                patch.object(
                    firmware_workspace.tempfile, "gettempdir", return_value=str(root / "user name")
                ),
            ):
                staged = firmware_workspace.staged_project(source)
            self.assertTrue(staged.is_relative_to(runtime.resolve()))

    def test_dependency_conflicts_fail_verification_before_native_checks(self) -> None:
        with (
            patch.object(
                setup_project.subprocess,
                "run",
                return_value=subprocess.CompletedProcess(
                    ["python", "-m", "pip", "check"],
                    1,
                    "mediapipe requires protobuf<5\ndepthai 3.10.0 is not supported on this platform\n",
                ),
            ),
            patch.object(setup_project, "run_python") as checks,
        ):
            with self.assertRaises(subprocess.CalledProcessError):
                setup_project.verify()
            checks.assert_not_called()

    def test_camera_metadata_warning_still_requires_native_camera_import(self) -> None:
        for version in ("3.10.0", "3.12.0"):
            with (
                self.subTest(version=version),
                patch.object(
                    setup_project.subprocess,
                    "run",
                    return_value=subprocess.CompletedProcess(
                        ["python", "-m", "pip", "check"],
                        1,
                        f"depthai {version} is not supported on this platform\n",
                    ),
                ),
                patch.object(setup_project, "run_python") as checks,
            ):
                setup_project.verify()
                self.assertIn(call("-c", "import depthai"), checks.call_args_list)
                for module in ("torch", "onnx", "platformio"):
                    self.assertIn(call("-c", f"import {module}"), checks.call_args_list)
                self.assertIn(call("-m", "platformio", "--version"), checks.call_args_list)
                self.assertIn(
                    call(str(ROOT / "scripts/setup_pose_models.py")), checks.call_args_list
                )

    def test_launch_does_not_install_or_verify_dependencies(self) -> None:
        root = ROOT / "clone with spaces and ?"
        with (
            patch.object(setup_project, "ROOT", root),
            patch.object(setup_project.subprocess, "call", return_value=7) as launch,
            patch.object(setup_project, "install") as install,
            patch.object(setup_project, "verify") as verify,
            patch.dict(os.environ),
        ):
            self.assertEqual(setup_project.main(["--launch"]), 7)
            launch.assert_called_once_with(
                [sys.executable, "-m", "aurora.gui.app"],
                cwd=root,
            )
            install.assert_not_called()
            verify.assert_not_called()
            self.assertEqual(os.environ["AURORA_SIMULATION_PYTHON"], sys.executable)

    def test_setup_includes_training_and_firmware_dependencies(self) -> None:
        requirements = (ROOT / "requirements-setup.txt").read_text(encoding="utf-8")
        extras = next(line for line in requirements.splitlines() if line.startswith("-e .["))
        groups = extras.removeprefix("-e .[").removesuffix("]").split(",")
        self.assertTrue({"training", "firmware"}.issubset(groups))

    def test_setup_checks_packages_and_prepares_firmware_before_success(self) -> None:
        with (
            patch.object(setup_project, "run_python") as run,
            patch.object(setup_project, "verify") as verify,
            patch.object(setup_project, "prepare_firmware") as firmware,
        ):
            steps = Mock()
            steps.attach_mock(run, "install")
            steps.attach_mock(verify, "verify")
            steps.attach_mock(firmware, "firmware")
            setup_project.install()
        self.assertEqual(
            steps.mock_calls,
            [
                call.install("-m", "pip", "install", "-r", "requirements-setup.txt"),
                call.verify(install_models=True),
                call.firmware(),
            ],
        )

    def test_setup_builds_in_a_mirror_without_uploading(self) -> None:
        with patch.object(setup_project, "run_python") as run:
            setup_project.prepare_firmware()
        run.assert_called_once_with(
            str(ROOT / "src/aurora/gui/firmware_workspace.py"),
            "--project",
            str(ROOT / "firmware/high_rate_emg_imu"),
            "--",
            sys.executable,
            "-m",
            "platformio",
            "run",
        )

    def test_pose_model_failure_stops_setup_before_firmware_or_launch(self) -> None:
        model_setup = str(ROOT / "scripts/setup_pose_models.py")

        def run(*arguments):
            if arguments[0] == model_setup:
                self.assertEqual(arguments[1:], ("--install",))
                raise subprocess.CalledProcessError(1, list(arguments))

        output = io.StringIO()
        with (
            patch.object(setup_project, "run_python", side_effect=run),
            patch.object(
                setup_project.subprocess,
                "run",
                return_value=subprocess.CompletedProcess([], 0, "No broken requirements found.\n"),
            ),
            patch.object(setup_project, "prepare_firmware") as firmware,
            patch.object(setup_project.subprocess, "call") as launch,
            patch.dict(os.environ),
            contextlib.redirect_stdout(output),
            contextlib.redirect_stderr(output),
        ):
            self.assertEqual(setup_project.main(["--install", "--launch"]), 1)
        firmware.assert_not_called()
        launch.assert_not_called()
        self.assertIn("Setup failed:", output.getvalue())
        self.assertNotIn("Setup complete.", output.getvalue())

    def test_failed_firmware_build_fails_setup_without_launching(self) -> None:
        output = io.StringIO()
        with (
            patch.object(setup_project, "run_python"),
            patch.object(setup_project, "verify"),
            patch.object(
                setup_project,
                "prepare_firmware",
                side_effect=subprocess.CalledProcessError(1, ["platformio", "run"]),
            ),
            patch.object(setup_project.subprocess, "call") as launch,
            patch.dict(os.environ),
            contextlib.redirect_stdout(output),
            contextlib.redirect_stderr(output),
        ):
            self.assertEqual(setup_project.main(["--install", "--launch"]), 1)
        launch.assert_not_called()
        self.assertIn("Setup failed:", output.getvalue())
        self.assertNotIn("Setup complete.", output.getvalue())

    def test_gui_discovers_windows_cached_environment_before_system_python(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            state = root / ".aurora"
            state.mkdir()
            prefix = root / "cache"
            (state / "runtime.json").write_text(
                json.dumps({"prefix": str(prefix)}),
                encoding="utf-8-sig",
            )
            with (
                patch("aurora.gui.project_support.REPOSITORY_ROOT", root),
                patch.dict(os.environ, {"AURORA_SIMULATION_PYTHON": ""}),
            ):
                python = "python.exe" if os.name == "nt" else "bin/python"
                self.assertEqual(simulation_python_candidates()[0], (prefix / python).resolve())


if __name__ == "__main__":
    unittest.main()
