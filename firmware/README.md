# Firmware

`high_rate_emg_imu/` contains the ESP32-C3 firmware for EMG and two LSM6DSO32
IMUs. It sends sensor data over BLE and provides clock synchronization for
camera recordings. USB streaming is available for diagnostics.

Build and flash it from the app's Firmware tab, or use ESP-IDF or PlatformIO.
The firmware uses ESP-IDF and NimBLE.

See the [firmware setup](high_rate_emg_imu/README.md) for the commands and pins.
