# Antonia, Majd
"""List nearby BLE devices and their advertised services."""

from __future__ import annotations

import argparse
import asyncio


async def scan(seconds: float, name: str = "") -> bool:
    from bleak import BleakScanner

    devices = await BleakScanner.discover(timeout=seconds, return_adv=True)
    matches = 0
    for device, advertisement in devices.values():
        device_name = advertisement.local_name or device.name or "Unnamed device"
        if name and name.casefold() not in device_name.casefold():
            continue
        matches += 1
        print(f"{device_name} | {device.address} | RSSI {advertisement.rssi} dBm")
        print(f"  Services: {', '.join(advertisement.service_uuids) or 'none advertised'}")
        if advertisement.manufacturer_data:
            values = ", ".join(
                f"{identifier}: {payload.hex()}"
                for identifier, payload in advertisement.manufacturer_data.items()
            )
            print(f"  Manufacturer data: {values}")
    if not matches:
        print(f"No BLE devices found{f' matching {name!r}' if name else ''}.")
    return matches > 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--name", default="", help="Only show names containing this text.")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    try:
        from bleak.exc import BleakError
    except ImportError:
        print("BLE scan needs bleak. Run Setup first.")
        return 2
    try:
        return 0 if asyncio.run(scan(args.seconds, args.name)) else 1
    except (BleakError, OSError, RuntimeError) as error:
        print(f"BLE scan failed: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
