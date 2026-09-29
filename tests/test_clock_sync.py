"""Verify BLE device clocks and camera samples share one monotonic timeline."""

from __future__ import annotations

import csv
from pathlib import Path
import struct
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "recording"))

from clock_sync import ClockSynchronizer, SYNC_RESPONSE
from record_ble_sensors import BleRecorder


class ClockSynchronizerTests(unittest.TestCase):
    def test_affine_fit_recovers_offset_and_device_drift(self) -> None:
        synchronizer = ClockSynchronizer()
        offset_ns = 8_000_000_000_000
        rate = 1.000075

        def host_time(device_ns: int) -> int:
            return round(offset_ns + device_ns * rate)

        for index in range(8):
            device_rx_us = 1_000_000 + index * 2_000_000
            device_tx_us = device_rx_us + 100
            host_tx_ns = host_time(device_rx_us * 1000) - 1_000_000
            host_rx_ns = host_time(device_tx_us * 1000) + 1_000_000
            exchange_id, _ = synchronizer.make_request(host_tx_ns)
            response = SYNC_RESPONSE.pack(
                1, 0, exchange_id, host_tx_ns, device_rx_us, device_tx_us
            )
            self.assertIsNotNone(synchronizer.handle_response(response, host_rx_ns))

        target_us = 18_000_000
        self.assertAlmostEqual(
            synchronizer.to_host_ns(target_us),
            host_time(target_us * 1000),
            delta=2_000,
        )
        summary = synchronizer.summary()
        self.assertTrue(summary["available"])
        self.assertAlmostEqual(summary["drift_ppm"], 75.0, delta=1.0)
        self.assertEqual(summary["clock_domain"], "host_monotonic_ns")

    def test_invalid_or_slow_exchanges_are_rejected(self) -> None:
        synchronizer = ClockSynchronizer(max_round_trip_ms=10)
        exchange_id, _ = synchronizer.make_request(1_000_000_000)
        wrong_echo = SYNC_RESPONSE.pack(1, 0, exchange_id, 2, 100, 101)
        self.assertIsNone(synchronizer.handle_response(wrong_echo, 1_001_000_000))

        exchange_id, _ = synchronizer.make_request(2_000_000_000)
        too_slow = SYNC_RESPONSE.pack(
            1, 0, exchange_id, 2_000_000_000, 200, 201
        )
        self.assertIsNone(synchronizer.handle_response(too_slow, 2_020_000_000))
        self.assertFalse(synchronizer.ready)

    def test_dual_imu_csv_uses_acquisition_time_not_arrival_time(self) -> None:
        synchronizer = ClockSynchronizer()
        host_tx_ns = 5_000_000_000
        exchange_id, _ = synchronizer.make_request(host_tx_ns)
        synchronizer.handle_response(
            SYNC_RESPONSE.pack(1, 0, exchange_id, host_tx_ns, 999_000, 999_100),
            host_rx_ns=5_002_100_000,
        )

        with tempfile.TemporaryDirectory() as temporary:
            recorder = BleRecorder(Path(temporary), clock_sync=synchronizer)
            packet = struct.pack(
                "<HIQ12h", 0x0103, 7, 1_000_000, *range(12)
            )
            recorder.handle("imu", bytearray(packet))
            recorder.close()
            with (Path(temporary) / "imu.csv").open(
                newline="", encoding="utf-8"
            ) as source:
                rows = list(csv.DictReader(source))

        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["device_time"], "1000000")
        self.assertNotEqual(rows[0]["synced_host_monotonic_ns"], "")
        self.assertEqual(rows[0]["sensor"], "0")
        self.assertEqual(rows[1]["sensor"], "1")

    def test_current_packet_connected_mask_omits_a_missing_wrist(self) -> None:
        samples = []
        recorder = BleRecorder(
            None,
            lambda sensor, sequence, channels: samples.append(
                (sensor, sequence, channels)
            ),
        )
        recorder.handle(
            "imu",
            bytearray(struct.pack("<HIQ12h", 0x0101, 4, 100, *range(12))),
        )
        recorder.close()
        self.assertEqual(samples[0][0:2], ("imu", 4))
        self.assertEqual(len(samples[0][2]), 1)


if __name__ == "__main__":
    unittest.main()
