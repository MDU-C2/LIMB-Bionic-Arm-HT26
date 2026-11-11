# Christian och Majd
"""Check pose model installation without downloads or connected hardware."""

from __future__ import annotations

import contextlib
import io
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, call, patch

import numpy as np

import scripts.setup_pose_models as setup_pose_models

MODEL = b"\x10\x00\x00\x00TFL3test-model"


class PoseModelSetupTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="aurora pose Å ")
        self.addCleanup(temporary.cleanup)
        package = Path(temporary.name) / "mediapipe"
        self.directory = package / "modules/pose_landmark"
        self.directory.mkdir(parents=True)
        self.pose = MagicMock()
        module = types.SimpleNamespace(
            __file__=str(package / "__init__.py"),
            solutions=types.SimpleNamespace(pose=types.SimpleNamespace(Pose=self.pose)),
        )
        modules = patch.dict(sys.modules, {"mediapipe": module})
        modules.start()
        self.addCleanup(modules.stop)

    def model_path(self, name):
        return self.directory / f"pose_landmark_{name}.tflite"

    def write_models(self):
        for _complexity, name in setup_pose_models.POSE_MODELS:
            self.model_path(name).write_bytes(MODEL)

    def test_check_fails_without_downloading_or_loading_a_missing_model(self):
        self.model_path("full").write_bytes(MODEL)
        with (
            patch.object(setup_pose_models.urllib.request, "urlopen") as download,
            self.assertRaisesRegex(RuntimeError, "lite pose model is missing"),
        ):
            setup_pose_models.check_models()
        download.assert_not_called()
        self.pose.assert_not_called()

    def test_install_downloads_missing_variants_and_preserves_the_bundled_model(self):
        full = self.model_path("full")
        full.write_bytes(MODEL + b"bundled")
        with patch.object(
            setup_pose_models.urllib.request,
            "urlopen",
            side_effect=lambda *_args, **_kwargs: io.BytesIO(MODEL),
        ) as download:
            setup_pose_models.check_models(install=True)
            self.assertEqual(
                download.call_args_list,
                [
                    call(setup_pose_models.MODEL_URL + f"pose_landmark_{name}.tflite", timeout=60)
                    for name in ("lite", "heavy")
                ],
            )
            setup_pose_models.check_models(install=True)
            self.assertEqual(download.call_count, 2)
        self.assertEqual(full.read_bytes(), MODEL + b"bundled")
        self.assertFalse(list(self.directory.glob("*.download")))

    def test_check_loads_all_pose_options_and_processes_images_without_network(self):
        self.write_models()
        with patch.object(setup_pose_models.urllib.request, "urlopen") as download:
            setup_pose_models.check_models()
        download.assert_not_called()
        self.assertEqual(
            self.pose.call_args_list,
            [call(static_image_mode=True, model_complexity=index) for index in range(3)],
        )
        graph = self.pose.return_value.__enter__.return_value
        self.assertEqual(graph.process.call_count, 3)
        for args in graph.process.call_args_list:
            self.assertEqual(args.args[0].shape, (64, 64, 3))
            self.assertEqual(args.args[0].dtype, np.uint8)

    def test_interrupted_download_keeps_the_previous_file_and_can_be_retried(self):
        path = self.model_path("lite")
        path.write_bytes(b"previous file")

        def interrupted(source, target):
            target.write(source.read(8))
            raise OSError("Connection interrupted")

        with (
            patch.object(
                setup_pose_models.urllib.request,
                "urlopen",
                side_effect=lambda *_args, **_kwargs: io.BytesIO(MODEL),
            ),
            patch.object(setup_pose_models.shutil, "copyfileobj", side_effect=interrupted),
            self.assertRaisesRegex(OSError, "interrupted"),
        ):
            setup_pose_models.download_model(path)
        self.assertEqual(path.read_bytes(), b"previous file")
        self.assertFalse(list(self.directory.glob("*.download")))
        with patch.object(
            setup_pose_models.urllib.request, "urlopen", return_value=io.BytesIO(MODEL)
        ):
            setup_pose_models.download_model(path)
        self.assertEqual(path.read_bytes(), MODEL)

    def test_invalid_response_is_not_published_as_a_model(self):
        path = self.model_path("lite")
        with (
            patch.object(
                setup_pose_models.urllib.request,
                "urlopen",
                return_value=io.BytesIO(b"<html>Error</html>"),
            ),
            self.assertRaisesRegex(ValueError, "not a TFLite model"),
        ):
            setup_pose_models.download_model(path)
        self.assertFalse(path.exists())
        self.assertFalse(list(self.directory.glob("*.download")))

    def test_install_repairs_an_empty_model(self):
        self.write_models()
        path = self.model_path("lite")
        path.write_bytes(b"")
        with patch.object(
            setup_pose_models.urllib.request, "urlopen", return_value=io.BytesIO(MODEL)
        ) as download:
            setup_pose_models.check_models(install=True)
        self.assertEqual(download.call_count, 1)
        self.assertEqual(path.read_bytes(), MODEL)

    def test_graph_initialization_failure_fails_the_command(self):
        self.write_models()
        self.pose.side_effect = RuntimeError("Cannot load model")
        with contextlib.redirect_stderr(io.StringIO()) as output:
            self.assertEqual(setup_pose_models.main([]), 1)
        self.assertIn("Cannot load model", output.getvalue())


if __name__ == "__main__":
    unittest.main()
