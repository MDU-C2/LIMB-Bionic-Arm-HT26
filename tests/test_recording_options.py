# Majd
"""Check recorder options before a camera or serial device can be opened."""

from __future__ import annotations

import contextlib
import io
import os
import sys
import unittest
from unittest.mock import patch

from aurora.recording import (
    record_ble_sensors,
    record_high_rate_serial,
    record_oak_pose,
    record_serial_sensors,
)
from aurora.recording.common import env_float


class RecordingOptionTests(unittest.TestCase):
    def test_subject_id_and_earlier_subject_option_match(self):
        for recorder in (
            record_ble_sensors,
            record_high_rate_serial,
            record_oak_pose,
            record_serial_sensors,
        ):
            for option in ("--subject-id", "--subject"):
                with (
                    self.subTest(recorder=recorder.__name__, option=option),
                    patch.object(sys, "argv", [recorder.__name__, option, "S01"]),
                ):
                    self.assertEqual(recorder.parse_args().subject, "S01")

    def test_invalid_durations_stop_before_opening_devices(self):
        for recorder in (record_high_rate_serial, record_oak_pose, record_serial_sensors):
            for value in ("nan", "inf", "-1"):
                arguments = [recorder.__name__, "--duration", value]
                if recorder is not record_oak_pose:
                    arguments += ["--port", "unused"]
                with (
                    self.subTest(recorder=recorder.__name__, value=value),
                    patch.object(sys, "argv", arguments),
                    contextlib.redirect_stdout(io.StringIO()),
                    patch("serial.Serial") as serial,
                ):
                    self.assertEqual(recorder.main(), 2)
                    serial.assert_not_called()

    def test_nonfinite_environment_duration_is_rejected(self):
        for value in ("nan", "inf", "-1"):
            with (
                self.subTest(value=value),
                patch.dict(os.environ, {"AURORA_DURATION_SECONDS": value}),
                self.assertRaises(ValueError),
            ):
                env_float("AURORA_DURATION_SECONDS", 0)


if __name__ == "__main__":
    unittest.main()
