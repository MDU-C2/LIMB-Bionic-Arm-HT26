# Majd
"""Find the selected sensor board, including the earlier firmware name."""

from aurora.sensors.settings import DEFAULT_BLE_DEVICE_NAME, LEGACY_BLE_DEVICE_NAME


def matches_device(requested: str, device, advertisement) -> bool:
    names = {requested.casefold()}
    if requested.casefold() == DEFAULT_BLE_DEVICE_NAME.casefold():
        names.add(LEGACY_BLE_DEVICE_NAME.casefold())
    actual = advertisement.local_name or device.name or ""
    return actual.casefold() in names


async def find_device(scanner, requested: str, timeout: float):
    return await scanner.find_device_by_filter(
        lambda device, advertisement: matches_device(requested, device, advertisement),
        timeout=timeout,
    )
