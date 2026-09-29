# Firmware

`dual_imu_serial/` is the current ESP-IDF-only target for the ESP32-C3-Zero. It reads
the shoulder LSM6DSO32 at `0x6B` and wrist LSM6DSO32 at `0x6A` on one I2C bus,
then publishes acquisition-timestamped binary samples over ESP-IDF NimBLE and
keeps the role-named USB serial JSON stream for setup and diagnostics.

The source uses ESP-IDF drivers, FreeRTOS, NVS, and NimBLE directly. It contains
no Arduino framework or Arduino BLE dependency. Build it with native `idf.py`
or the checked-in PlatformIO environment, which is pinned to ESP-IDF 4.4.7.

The implementation follows the verified ESP-IDF IMU register setup in
LIMB-HT25. Generated build and editor files are intentionally not part of this
repository.

See [the sensor guide](../docs/SENSOR_DATA.md) and the
[firmware README](dual_imu_serial/README.md).
