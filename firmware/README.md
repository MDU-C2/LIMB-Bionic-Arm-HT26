# Firmware

`dual_imu_serial/` is the only maintained ESP-IDF target for the ESP32-C3-Zero.
It reads the shoulder LSM6DSO32 at `0x6B`, wrist LSM6DSO32 at `0x6A`, and the
EMG analogue output on ADC1 channel 0 / GPIO0. It publishes all three on one
acquisition clock over ESP-IDF NimBLE and keeps role-named USB JSON for setup,
live simulation, and diagnostics.

The source uses ESP-IDF drivers, FreeRTOS, NVS, and NimBLE directly. It contains
no Arduino framework or Arduino BLE dependency. Build it with native `idf.py`
or the checked-in PlatformIO environment, which is pinned to ESP-IDF 4.4.7.

The implementation follows the verified ESP-IDF IMU register setup in
LIMB-HT25. Generated build and editor files are intentionally not part of this
repository.

See [the sensor guide](../docs/SENSOR_DATA.md) and the
[firmware README](dual_imu_serial/README.md).
