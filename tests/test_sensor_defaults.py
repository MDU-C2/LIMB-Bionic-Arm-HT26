# Majd
"""Check current defaults and connections to earlier sensor firmware."""

from __future__ import annotations

import asyncio
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

from aurora.sensors.ble_discovery import find_device, matches_device
from aurora.sensors.live_input import LiveSensorInput
from aurora.sensors.settings import DEFAULT_BLE_DEVICE_NAME, DEFAULT_GRIP_SOURCE
from aurora.simulation import live_control
from aurora.simulation.interactive.settings import parse_args


class SensorDefaultTests(unittest.TestCase):
    def test_current_and_earlier_firmware_names_are_accepted(self):
        for name in ("AURORA", "aurora", "LIMBServer"):
            with self.subTest(name=name):
                self.assertTrue(
                    matches_device(
                        "AURORA", SimpleNamespace(name=None), SimpleNamespace(local_name=name)
                    )
                )
        self.assertFalse(
            matches_device(
                "AURORA", SimpleNamespace(name="Other"), SimpleNamespace(local_name=None)
            )
        )

    def test_an_explicit_device_name_does_not_select_another_board(self):
        self.assertFalse(
            matches_device(
                "Subject02", SimpleNamespace(name="AURORA"), SimpleNamespace(local_name=None)
            )
        )
        self.assertTrue(
            matches_device(
                "Subject02", SimpleNamespace(name="Subject02"), SimpleNamespace(local_name=None)
            )
        )

    def test_discovery_uses_the_same_filter_for_recording_and_live_input(self):
        scanner = SimpleNamespace(find_device_by_filter=AsyncMock(return_value="old-board"))
        self.assertEqual(asyncio.run(find_device(scanner, "AURORA", 3)), "old-board")
        callback = scanner.find_device_by_filter.call_args.args[0]
        self.assertTrue(
            callback(SimpleNamespace(name="LIMBServer"), SimpleNamespace(local_name=None))
        )
        self.assertEqual(scanner.find_device_by_filter.call_args.kwargs["timeout"], 3)

    def test_command_line_and_sensor_input_share_emg_and_device_defaults(self):
        with patch("sys.argv", ["live_control"]):
            launch = live_control.parse_args()
        interactive = parse_args([])
        reader = LiveSensorInput()
        self.assertEqual(DEFAULT_GRIP_SOURCE, "emg")
        self.assertEqual(DEFAULT_BLE_DEVICE_NAME, "AURORA")
        for settings in (launch, interactive, reader):
            self.assertEqual(settings.grip_source, DEFAULT_GRIP_SOURCE)
            self.assertEqual(settings.device, DEFAULT_BLE_DEVICE_NAME)


if __name__ == "__main__":
    unittest.main()
