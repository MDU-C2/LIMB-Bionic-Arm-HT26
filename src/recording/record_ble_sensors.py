# Majd, Christian
"""Record synchronized AURORA IMU and EMG notifications over BLE."""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import time

from ble_dataset import LabeledBleCapture
from ble_packets import CURRENT_EMG_PACKET as CURRENT_EMG_PACKET
from ble_packets import DEFAULT_SENSORS as DEFAULT_SENSORS
from ble_packets import HIGH_RATE_EMG_HEADER as HIGH_RATE_EMG_HEADER
from ble_packets import HIGH_RATE_IMU_PACKET as HIGH_RATE_IMU_PACKET
from ble_packets import IMU_COLUMNS as IMU_COLUMNS
from ble_packets import LEGACY_EMG_PACKET as LEGACY_EMG_PACKET
from ble_packets import LEGACY_IMU_PACKET as LEGACY_IMU_PACKET
from ble_packets import SENSOR_UUIDS as SENSOR_UUIDS

# Continuous ADC packets have a 22-byte header followed by up to 40 samples.
from ble_packets import SERVICE_UUID as SERVICE_UUID
from ble_packets import TIMESTAMPED_PACKETS as TIMESTAMPED_PACKETS
from ble_packets import BleRecorder as BleRecorder
from clock_sync import TIME_SYNC_UUID, BleClockSyncClient, ClockSynchronizer
from common import (
    create_session_directory,
    enable_ctrl_break_as_interrupt,
    env_float,
    experiment_metadata,
    write_metadata,
)
from emg_trials import GripTrial


def parse_args() -> argparse.Namespace:
    """Read GUI defaults and optional command-line overrides."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=os.environ.get("AURORA_BLE_DEVICE", "LIMBServer"))
    parser.add_argument("--address", default="", help="Connect to a known BLE address directly.")
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show live EMG and IMU feedback without saving a session.",
    )
    parser.add_argument(
        "--preview-sensor",
        choices=("all", "emg", "imu", "piezo"),
        default="all",
        help="Choose which BLE sensor window to open first.",
    )
    parser.add_argument(
        "--sensors",
        nargs="+",
        choices=tuple(SENSOR_UUIDS),
        default=None,
        help=(
            "Sensor streams to request. Each characteristic is tried independently, "
            "so an unavailable sensor does not stop the others."
        ),
    )
    parser.add_argument("--scan-timeout", type=float, default=12.0)
    parser.add_argument("--subject", default=os.environ.get("AURORA_SUBJECT", "session"))
    parser.add_argument(
        "--duration",
        type=float,
        default=env_float("AURORA_DURATION_SECONDS", 0.0),
        help="Seconds to record; 0 records until stopped (labeled capture auto-stops).",
    )
    parser.add_argument(
        "--dataset-label",
        default="",
        help="Capture 80 labeled 100 ms windows (20 rest, 40 movement, 20 rest).",
    )
    parser.add_argument("--grip-trial", action="store_true", help="Record guided grip phases.")
    parser.add_argument("--grip-repeats", type=int, default=5, help="Grip repeats, from 1 to 20.")
    return parser.parse_args()


async def record(args: argparse.Namespace) -> int:
    """Connect to a LIMB server and record every available sensor stream."""
    requested_value = getattr(args, "sensors", None)
    requested = (
        DEFAULT_SENSORS if requested_value is None else tuple(dict.fromkeys(requested_value))
    )
    grip_trial = None
    if getattr(args, "grip_trial", False):
        if args.dataset_label or getattr(args, "preview", False) or "emg" not in requested:
            print("A grip trial needs EMG recording with movement labels and preview off.")
            return 2
        try:
            grip_trial = GripTrial(getattr(args, "grip_repeats", 5))
        except ValueError as error:
            print(error)
            return 2
        if args.duration and args.duration < grip_trial.duration + 3:
            print(f"Use duration 0 or at least {grip_trial.duration + 3:g} seconds for this trial.")
            return 2
    if not requested or any(name not in SENSOR_UUIDS for name in requested):
        print("Choose at least one sensor: imu or emg (piezo is legacy-only).")
        return 2
    if getattr(args, "preview", False):
        if args.dataset_label:
            print("Preview cannot be combined with labeled capture.")
            return 2
        from ble_preview import run_ble_preview

        def preview_recorder(session, sample_sink):
            return BleRecorder(session, sample_sink, sensors=requested)

        return await run_ble_preview(
            args,
            preview_recorder,
            {name: SENSOR_UUIDS[name] for name in requested},
        )
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("bleak is missing. Recreate the simulation environment.")
        return 2

    if (
        not math.isfinite(args.duration)
        or not math.isfinite(args.scan_timeout)
        or args.duration < 0
        or args.scan_timeout <= 0
    ):
        print("Duration cannot be negative and scan timeout must be positive.")
        return 2
    try:
        dataset_label = LabeledBleCapture.validate_label(args.dataset_label)
    except ValueError as error:
        print(error)
        return 2

    print(f"Scanning for {args.device!r}...")
    device = args.address or await BleakScanner.find_device_by_name(
        args.device,
        timeout=args.scan_timeout,
    )
    if not device:
        print(f"BLE device {args.device!r} was not found.")
        return 1

    session = create_session_directory("ble", args.subject)
    capture = LabeledBleCapture(session, dataset_label, args.subject) if dataset_label else None
    clock_sync = ClockSynchronizer()
    recorder = BleRecorder(
        session,
        capture.add_packet if capture is not None else None,
        clock_sync,
        requested,
    )
    started = time.monotonic()
    subscribed: list[str] = []
    unavailable: dict[str, str] = {}
    sync_client: BleClockSyncClient | None = None
    grip_summary: dict = {}
    last_emg_count = 0
    last_emg_time = started
    print(f"Saving to {session}")

    try:
        async with BleakClient(device) as client:
            print(f"Connected to {getattr(device, 'address', device)}")
            sync_client = BleClockSyncClient(client, clock_sync)
            sync_available = await sync_client.start()
            if sync_available:
                print(
                    "Device clock synchronized to host monotonic time "
                    f"({len(clock_sync.samples)} exchanges)"
                )
            else:
                print("Time-sync characteristic unavailable; preserving arrival times only")
            for sensor in requested:
                uuid = SENSOR_UUIDS[sensor]
                try:
                    await client.start_notify(
                        uuid,
                        lambda _sender, data, name=sensor: recorder.handle(name, data),
                    )
                    subscribed.append(sensor)
                    print(f"Recording {sensor.upper()}")
                except Exception as error:
                    unavailable[sensor] = str(error)
                    print(f"{sensor.upper()} is unavailable: {error}")

            if not subscribed:
                print("The device has none of the known LIMB sensor characteristics.")
                return 1

            if grip_trial is not None:
                if "emg" not in subscribed or not clock_sync.ready:
                    print("A grip trial needs EMG and device clock sync. Check the firmware.")
                    return 1
                for remaining in (3, 2, 1):
                    print(f"[CAPTURE] Grip trial starts in {remaining}")
                    await asyncio.sleep(1.0)
                if recorder.counts["emg"] == 0:
                    print("No EMG samples received. Check the BLE stream.")
                    return 1
                print(f"[CAPTURE] {grip_trial.advance(time.monotonic_ns())}")

            # Count the requested duration from sensor subscription, after connection and sync.
            started = time.monotonic()
            if capture is not None:
                if not {"emg", "imu"}.issubset(subscribed):
                    print("Labeled capture needs both EMG and IMU characteristics.")
                    return 1
                print(
                    f"Labeled capture: 20 rest, 40 label {dataset_label}, 20 rest "
                    "(80 windows total)."
                )
                for remaining in (3, 2, 1):
                    print(f"Starting in {remaining}...")
                    await asyncio.sleep(1.0)
                capture.start()
                started = time.monotonic()
                print("Recording now. Keep the first 20 windows at rest.")

            while args.duration == 0 or time.monotonic() - started < args.duration:
                await asyncio.sleep(0.02 if grip_trial is not None else 1.0)
                if not client.is_connected:
                    raise RuntimeError("BLE disconnected; the recording stopped.")
                if grip_trial is not None:
                    if recorder.counts["emg"] != last_emg_count:
                        last_emg_count = recorder.counts["emg"]
                        last_emg_time = time.monotonic()
                    elif time.monotonic() - last_emg_time > 2:
                        raise RuntimeError("EMG samples stopped during the grip trial.")
                    cue = grip_trial.advance(time.monotonic_ns())
                    if cue:
                        print(f"[CAPTURE] {cue}")
                    if grip_trial.complete:
                        break
                    continue
                summary = ", ".join(
                    f"{name} {recorder.counts[name]} (missing {recorder.missing_packets[name]})"
                    for name in subscribed
                )
                print(f"Packets: {summary}")
                if capture is not None:
                    if capture.emg_complete:
                        break
                    if capture.seconds_since_progress > 10.0:
                        print("No complete EMG window for 10 seconds; capture is incomplete.")
                        break
    finally:
        if sync_client is not None:
            await sync_client.close()
        recorder.close()
        if grip_trial is not None:
            grip_summary = grip_trial.save(session, time.monotonic_ns())
        if capture is not None:
            capture.save()
        write_metadata(
            session,
            {
                "source": "ble",
                "subject": args.subject,
                "device": args.device,
                "service_uuid": SERVICE_UUID,
                "characteristics": {name: SENSOR_UUIDS[name] for name in requested},
                "requested_sensors": list(requested),
                "subscribed_sensors": subscribed,
                "unavailable_sensors": unavailable,
                "time_sync_characteristic": TIME_SYNC_UUID,
                "time_sync": clock_sync.summary(),
                "duration_seconds": round(time.monotonic() - started, 3),
                "packet_counts": recorder.counts,
                "packet_health": recorder.packet_health(),
                **experiment_metadata(),
                **({"labeled_capture": capture.summary()} if capture is not None else {}),
                **({"grip_trial": grip_summary} if grip_trial is not None else {}),
            },
        )

    print(f"Saved BLE session to {session}")
    if grip_trial is not None and (not grip_trial.complete or not grip_summary["features_saved"]):
        print("Grip trial incomplete. Raw samples and cue times were saved.")
        return 1
    if capture is not None and not capture.complete:
        print("Labeled capture is incomplete; review the raw files before using it.")
        return 1
    return 0


def main() -> int:
    """Run the asynchronous recorder with clean Ctrl+C handling."""
    enable_ctrl_break_as_interrupt()
    args = parse_args()
    try:
        from bleak.exc import BleakError
    except ImportError:
        print("bleak is missing. Recreate the simulation environment.")
        return 2
    try:
        return asyncio.run(record(args))
    except KeyboardInterrupt:
        print("Stopping BLE recording...")
        return 0
    except (BleakError, OSError, RuntimeError) as error:
        print(f"BLE recording failed: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
