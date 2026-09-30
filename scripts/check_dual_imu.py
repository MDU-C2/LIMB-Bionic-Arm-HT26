"""Verify live shoulder/wrist IMUs and optional EMG from the ESP32 stream."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "recording"))

from emg_protocol import extract_emg
from imu_protocol import ImuStreamDecoder, extract_imus, i2c_wiring_hint


ROLE_ORDER = ("shoulder", "wrist")
ESPRESSIF_VID = 0x303A


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        help="Serial port such as COM5. Auto-detected when one ESP32 is attached.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--seconds",
        type=float,
        default=8.0,
        help="Maximum time to wait; exits early once both IMUs have enough samples.",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=5,
        help="Complete packets required from each IMU.",
    )
    parser.add_argument(
        "--require-emg",
        action="store_true",
        help="Also require valid GPIO0 ADC readings from the integrated EMG input.",
    )
    return parser.parse_args()


def select_port(requested: str | None) -> str:
    if requested:
        return requested

    from serial.tools import list_ports

    ports = list(list_ports.comports())
    esp32_ports = [port for port in ports if port.vid == ESPRESSIF_VID]
    candidates = esp32_ports or ports
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        raise RuntimeError("No serial device found. Connect the ESP32 over USB.")
    names = ", ".join(port.device for port in candidates)
    raise RuntimeError(f"More than one serial device is available ({names}); pass --port.")


def _valid_axes(value: object) -> bool:
    if not isinstance(value, dict):
        return False
    return all(
        isinstance(value.get(axis), (int, float))
        and not isinstance(value.get(axis), bool)
        and math.isfinite(float(value[axis]))
        for axis in ("x", "y", "z")
    )


def _format_axes(value: dict[str, Any]) -> str:
    return " ".join(f"{axis.upper()}={float(value[axis]):+.4f}" for axis in ("x", "y", "z"))


def verify_stream(
    port: str,
    baud: int,
    seconds: float,
    min_samples: int,
    require_emg: bool = False,
) -> bool:
    import serial

    decoder = ImuStreamDecoder()
    counts = {role: 0 for role in ROLE_ORDER}
    latest: dict[str, dict[str, Any]] = {}
    latest_packet: dict[str, Any] | None = None
    current_bus: dict[str, Any] | None = None
    packet_count = 0
    emg_count = 0
    latest_emg: dict[str, Any] = {}
    deadline = time.monotonic() + seconds

    print(f"Reading {port} at {baud} baud for up to {seconds:g} s...")
    with serial.Serial(port, baud, timeout=0.25) as device:
        device.reset_input_buffer()
        while time.monotonic() < deadline:
            raw = device.readline()
            if not raw:
                continue
            packet = decoder.feed(raw.decode("utf-8", errors="replace"))
            if not isinstance(packet, dict):
                continue
            packet_count += 1
            latest_packet = packet
            i2c = packet.get("i2c")
            if isinstance(i2c, dict) and (
                i2c.get("sda_pin"), i2c.get("scl_pin")
            ) == (2, 1):
                current_bus = i2c
            for role, sensor in extract_imus(packet).items():
                if (
                    role in counts
                    and sensor.get("connected") is True
                    and _valid_axes(sensor.get("accel_g"))
                    and _valid_axes(sensor.get("gyro_dps"))
                ):
                    counts[role] += 1
                    latest[role] = sensor
            emg = extract_emg(packet)
            if emg.get("connected") is True and "adc_raw" in emg:
                emg_count += 1
                latest_emg = emg
            if (
                all(counts[role] >= min_samples for role in ROLE_ORDER)
                and (not require_emg or emg_count >= min_samples)
            ):
                break

    if packet_count == 0:
        print("FAIL: the port opened, but no supported IMU packets were received.")
        return False

    success = True
    sensors = extract_imus(latest_packet)
    for role in ROLE_ORDER:
        sensor = latest.get(role)
        if sensor is None:
            success = False
            last = sensors.get(role, {})
            detail = last.get("error", "no complete acceleration/gyro packet")
            print(f"FAIL: {role:8s} {last.get('address', '?')} - {detail}")
            continue
        print(
            f"PASS: {role:8s} {sensor.get('address', '?')} "
            f"accel[g] {_format_axes(sensor['accel_g'])}  "
            f"gyro[dps] {_format_axes(sensor['gyro_dps'])}  "
            f"({counts[role]} packets)"
        )

    if latest_emg:
        print(
            f"PASS: emg      GPIO{latest_emg.get('gpio', '?')} "
            f"raw={latest_emg['adc_raw']} ADC  ({emg_count} packets)"
        )
    elif require_emg:
        success = False
        print("FAIL: emg      no valid ADC sample")

    if not success:
        hint = i2c_wiring_hint(current_bus or (latest_packet or {}).get("i2c"))
        if hint:
            print(f"WIRING: {hint}")
        print(
            "Expected shared bus: both SDA -> GPIO2, both SCL -> GPIO1, shared "
            "3.3 V/GND; shoulder SA0 high (0x6B), wrist SA0 low (0x6A)."
        )
    return success


def main() -> int:
    args = parse_args()
    if args.baud < 1 or args.seconds <= 0 or args.min_samples < 1:
        raise ValueError("baud, seconds, and min-samples must be positive")
    port = select_port(args.port)
    return (
        0
        if verify_stream(
            port,
            args.baud,
            args.seconds,
            args.min_samples,
            args.require_emg,
        )
        else 1
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(2) from error
