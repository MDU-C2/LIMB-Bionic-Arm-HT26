# Firmware

`dual_imu_serial/` is the current ESP-IDF target for the ESP32-C3-Zero. It reads
the shoulder LSM6DSO32 at `0x6A` and wrist LSM6DSO32 at `0x6B` on one I2C bus,
then sends one role-named JSON packet over USB serial.

The implementation follows the verified ESP-IDF IMU register setup in
LIMB-HT25. Generated build and editor files are intentionally not part of this
repository.

See [the sensor guide](../docs/SENSOR_DATA.md) and the
[firmware README](dual_imu_serial/README.md).
