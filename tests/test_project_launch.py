# Majd
"""Check installed entry points from a different working directory."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class ProjectLaunchTests(unittest.TestCase):
    def test_direct_file_help_does_not_need_the_package_or_a_display(self):
        app = Path(__file__).resolve().parents[1] / "src/aurora/gui/app.py"
        result = subprocess.run(
            [sys.executable, "-S", str(app), "--help"],
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("usage:", result.stdout)

    def test_direct_file_launch_works_before_the_package_is_installed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "checkout with spaces \u00c5"
            root.mkdir()
            app = root / "src/aurora/gui/app.py"
            app.parent.mkdir(parents=True)
            original = Path(__file__).resolve().parents[1] / "src/aurora/gui/app.py"
            app.write_bytes(original.read_bytes())
            if os.name == "nt":
                launcher = root / "scripts/setup.ps1"
                launcher.parent.mkdir()
                launcher.write_text(
                    'param([switch]$Start)\nif (-not $Start) { exit 9 }\nSet-Content -LiteralPath (Join-Path $PSScriptRoot "../started.txt") -Value "started"\nexit 7\n',
                    encoding="utf-8",
                )
            else:
                (root / "Start.command").write_text(
                    "#!/bin/bash\nprintf started > started.txt\nexit 7\n", encoding="utf-8"
                )
            result = subprocess.run(
                [sys.executable, "-S", str(app)],
                cwd=temporary,
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
            self.assertEqual(result.returncode, 7, result.stdout + result.stderr)
            self.assertEqual((root / "started.txt").read_text().strip(), "started")
            self.assertNotIn("ModuleNotFoundError", result.stderr)

    def test_entry_points_do_not_need_the_source_directory_on_sys_path(self):
        with tempfile.TemporaryDirectory() as temporary:
            for module in ("aurora.simulation.interactive", "aurora.recording.capture_movement"):
                with self.subTest(module=module):
                    result = subprocess.run(
                        [sys.executable, "-I", "-m", module, "--help"],
                        cwd=temporary,
                        capture_output=True,
                        text=True,
                        timeout=20,
                        check=False,
                    )
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn("usage:", result.stdout)

    def test_shared_paths_refer_to_the_checkout_from_any_working_directory(self):
        code = (
            "import json; from aurora.gui.project_support import "
            "REPOSITORY_ROOT, DOCUMENTATION_FILE, INTERACTIVE_SIMULATION_SCRIPT, "
            "POSE_RECORDING_SCRIPT, MOVEMENT_RECOGNITION_SCRIPT; "
            "from aurora.sensors.camera.cup_detection import MODEL_PATH; "
            "print(json.dumps([str(p) for p in (REPOSITORY_ROOT, DOCUMENTATION_FILE, "
            "INTERACTIVE_SIMULATION_SCRIPT, POSE_RECORDING_SCRIPT, "
            "MOVEMENT_RECOGNITION_SCRIPT, MODEL_PATH)]))"
        )
        with tempfile.TemporaryDirectory() as temporary:
            result = subprocess.run(
                [sys.executable, "-I", "-c", code],
                cwd=temporary,
                capture_output=True,
                text=True,
                encoding="utf-8",
                timeout=20,
                check=False,
            )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        paths = [Path(value) for value in json.loads(result.stdout)]
        self.assertEqual(paths[0], Path(__file__).resolve().parents[1])
        for path in paths[1:]:
            self.assertTrue(path.is_file(), path)


if __name__ == "__main__":
    unittest.main()
