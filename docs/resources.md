# Resources

References for the software and hardware used in AURORA and the earlier LIMB
project. See [hardware checks](HARDWARE_VALIDATION.md) for measurements on the
current setup and [motors](MOTORS.md) for the arm's motor assignments.

## Software and communication

| Reference | Use |
| --- | --- |
| [LIMB-HT25](https://github.com/MDU-C2/LIMB-HT25) | Earlier software, firmware and hardware designs |
| [Python](https://docs.python.org/3/) | Language and standard library |
| [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/index.html) | ESP32-C3 setup and drivers |
| [TWAI](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/peripherals/twai.html) | ESP32 CAN driver |
| [CAN in Automation](https://www.can-cia.org/can-knowledge/) | CAN bus fundamentals |
| [SocketCAN](https://www.kernel.org/doc/html/latest/networking/can.html) | Linux CAN interface |
| [python-can](https://python-can.readthedocs.io/en/stable/) | CAN messages from Python |
| [Bluetooth specification](https://www.bluetooth.com/specifications/specs/core-specification/) | BLE protocol |
| [Bleak](https://bleak.readthedocs.io/en/latest/) | Python BLE client |
| [Jetson AGX Orin](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/latest/index.html) | Earlier arm-control computer |
| [Ijspeert et al., Dynamical Movement Primitives](https://doi.org/10.1162/NECO_a_00393) | DMP theory |

## Hardware

Motor and CAN component names below come from the earlier BOM and firmware.
Check fitted parts and wiring before using their specifications.

| Reference | Part |
| --- | --- |
| [Espressif ESP32-C3](https://www.espressif.com/en/products/socs/esp32-c3) | Microcontroller |
| [Waveshare ESP32-C3-Zero](https://www.waveshare.com/esp32-c3-zero.htm) | Sensor controller board |
| [ST LSM6DSO32](https://www.st.com/en/mems-and-sensors/lsm6dso32.html) | Accelerometer and gyroscope |
| [JX PDI-HV2060MG](https://servodatabase.com/servo/jx-servo/pdi-hv2060mg) | Shoulder servos |
| [NEMA17 / 42SHD0217-24B](https://www.electrokit.com/upload/quick/ea/2d/5b25_41032662-tds.pdf) | Stepper motor |
| [NEMA23 / 57SHD4934-34B](https://www.electrokit.com/upload/quick/e6/0c/2f07_41032667-tds.pdf) | Motor listed in the BOM; assignment unresolved |
| [Pololu DRV8825](https://www.pololu.com/product/2133) | Stepper driver |
| [Hitec HS-422](https://www.electrokit.com/upload/product/41002/41002565/hs422.pdf) | Finger servos |
| [Whadda WPK601](https://www.velleman.eu/products/view/270-robot-digital-double-shaft-servo-kit-wpk601/?country=be&id=460528&lang=en) | Wrist rotation servo |
| [TI SN65HVD232](https://www.ti.com/product/SN65HVD232) | CAN transceiver |

EMG uses GPIO0 / ADC1 channel 0. The analogue board's model and gain still need
checking against the fitted circuit. Earlier pressure, potentiometer and piezo
inputs are not assumed to be present on the current arm.
