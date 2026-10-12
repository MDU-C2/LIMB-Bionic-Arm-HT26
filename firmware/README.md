# Firmware

[`high_rate_emg_imu/`](high_rate_emg_imu/README.md) is the maintained ESP32-C3
target. It acquires continuous EMG and up to two LSM6DSO32 IMUs, streams USB
and BLE data, and exposes BLE clock synchronization for OAK-D recordings.

The source uses ESP-IDF drivers, FreeRTOS, NVS, and NimBLE directly. It contains
no Arduino framework or Arduino BLE dependency. Build it with native `idf.py`
or the checked-in ESP-IDF 5.x PlatformIO environment.

It is based on the Jonas-AURORA source snapshot and incorporates the earlier
AURORA single-IMU wiring fallback. The IMU component credits Cornelia and
Jonas. Generated build and editor files are intentionally not part of this
repository.

See [the sensor guide](../docs/SENSOR_DATA.md) and the
[firmware README](high_rate_emg_imu/README.md).
