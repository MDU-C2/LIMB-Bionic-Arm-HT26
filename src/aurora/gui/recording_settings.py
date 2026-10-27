# Majd, Christian
"""Recording sources and source-specific command arguments."""

from __future__ import annotations

MAINTAINED_SOURCES = (
    (
        "record_oak_pose.py",
        "OAK-D camera",
        "Save video, body/arm pose, and selected-hand landmarks.",
    ),
    (
        "record_high_rate_serial.py",
        "ESP32 sensors (USB diagnostic)",
        "For diagnostics with USB streaming enabled in the firmware.",
    ),
    (
        "record_ble_sensors.py",
        "ESP32 sensors (Bluetooth)",
        "Save both IMUs and EMG on the same acquisition clock.",
    ),
)


DEFAULT_BATCH_SOURCES = {"record_ble_sensors.py", "record_oak_pose.py"}


DEFAULT_BLE_SENSORS = ("imu", "emg")


BLE_SENSOR_OPTIONS = (
    ("imu", "IMU"),
    ("emg", "EMG"),
    ("piezo", "Piezo (legacy firmware)"),
)


def batch_arguments(
    filename: str,
    camera_arguments: list[str],
    dataset_label: str = "",
    ble_sensors: tuple[str, ...] = (),
    grip_trial: bool = False,
    grip_repeats: int = 5,
) -> list[str]:
    """Build arguments for the selected recorder."""
    if filename == "record_oak_pose.py":
        return ["--auto-start", *camera_arguments]
    if filename == "record_ble_sensors.py":
        if grip_trial and (dataset_label or not 1 <= grip_repeats <= 20):
            raise ValueError("Choose 1 to 20 grip repeats and leave movement labels off.")
        arguments = ["--sensors", *ble_sensors] if ble_sensors else []
        if dataset_label:
            arguments.extend(("--dataset-label", dataset_label))
        if grip_trial:
            arguments.extend(("--grip-trial", "--grip-repeats", str(grip_repeats)))
        return arguments
    return []
