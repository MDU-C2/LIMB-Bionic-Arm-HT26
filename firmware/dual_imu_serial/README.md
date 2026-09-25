# Dual-IMU ESP32-C3 firmware

This ESP-IDF target replaces the temporary single-IMU scanner. It keeps the
verified GPIO 8/5 wiring from the current project and the LSM6DSO32 register
setup/scaling from LIMB-HT25.

Both IMUs share one I2C bus and must use different addresses:

| Role | LSM6DSO32 address | SA0/SDO |
| --- | --- | --- |
| Shoulder | `0x6A` | Low / GND |
| Wrist | `0x6B` | High / 3.3 V |

Connect both SDA pins to ESP32-C3 GPIO 8, both SCL pins to GPIO 5, and share
3.3 V and ground. Do not connect two sensors with the same address to this bus.

From an ESP-IDF terminal:

```powershell
cd firmware/dual_imu_serial
idf.py set-target esp32c3
idf.py build
idf.py -p COM5 flash monitor
```

The device emits one `aurora.dual_imu.v1` JSON line every 20 ms at 115200 baud.
The GUI labels the two entries as shoulder and wrist rather than relying on
ambiguous `imu1`/`imu2` names.
