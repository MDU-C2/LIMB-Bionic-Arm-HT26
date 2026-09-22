# Development Resources

This page collects the main references likely to be needed while evaluating and migrating LIMB work. Links favor project, vendor, and standards-maintainer sources.

**Status meaning:** **Confirmed** means the technology or model is explicitly referenced by the old LIMB repository, not that it has been verified on the current AURORA hardware. **Needs verification** means the exact device must be checked against the physical arm. **Recommended reference** is general development guidance.

| Resource | What it is used for | Link | Status |
| --- | --- | --- | --- |
| Old LIMB repository | Original software, firmware, hardware designs, documentation, tests, and research to evaluate before migration. | [MDU-C2/LIMB-HT25](https://github.com/MDU-C2/LIMB-HT25) | Confirmed |
| ESP32-C3 documentation | Microcontroller datasheet and technical information for the chip used by the legacy controller boards. | [Espressif ESP32-C3 documentation](https://www.espressif.com/en/products/socs/esp32-c3) | Confirmed |
| ESP-IDF programming guide | Setup, build system, APIs, and development workflow for ESP32-C3 firmware. Confirm the required ESP-IDF version before building old firmware. | [ESP-IDF Get Started for ESP32-C3](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/index.html) | Confirmed |
| ESP32-C3 TWAI driver | ESP-IDF interface for the ESP32's CAN-compatible controller, including errors and bus recovery. | [ESP-IDF TWAI documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/peripherals/twai.html) | Confirmed |
| CAN fundamentals | Overview of Classical CAN, CAN FD, CAN XL, and the protocol layers maintained by CAN in Automation. LIMB documents Classical CAN for the arm. | [CAN in Automation knowledge base](https://www.can-cia.org/can-knowledge/) | Recommended reference |
| SocketCAN | Linux CAN networking interface used by the legacy host-side communication code. | [Linux kernel SocketCAN documentation](https://www.kernel.org/doc/html/latest/networking/can.html) | Confirmed |
| Python | Language reference and standard-library documentation for computer-side development. | [Official Python documentation](https://docs.python.org/3/) | Recommended reference |
| `python-can` | Python interface used by LIMB to send and receive CAN messages through SocketCAN. | [`python-can` documentation](https://python-can.readthedocs.io/en/stable/) | Confirmed |
| Bluetooth Core Specification | Authoritative reference for Bluetooth Low Energy concepts and protocol behavior. | [Bluetooth SIG Core Specification](https://www.bluetooth.com/specifications/specs/core-specification/) | Recommended reference |
| Bleak | Python BLE client used by legacy host and data-capture software. | [Bleak documentation](https://bleak.readthedocs.io/en/latest/) | Confirmed |
| Jetson AGX Orin Developer Kit | Legacy documentation identifies this as the main computer used to run host control and SocketCAN. Confirm that it is still AURORA's computer and record its exact software image. | [NVIDIA Jetson AGX Orin Developer Kit guide](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/latest/index.html) | Needs verification |
| Dynamic Movement Primitives | Foundational reference for understanding the DMP trajectory work in the old repository. | [Ijspeert et al., “Dynamical Movement Primitives”](https://doi.org/10.1162/NECO_a_00393) | Recommended reference |
| ESP32-C3-Zero board | Small ESP32-C3 controller board named by the legacy hardware documentation. Confirm every installed controller before using its pinout. | [Waveshare ESP32-C3-Zero](https://www.waveshare.com/esp32-c3-zero.htm) | Needs verification |
| JX PDI-HV2060MG servo | Two units appear in the arm BOM and the firmware assigns them to two shoulder axes after modification for continuous rotation. | [PDI-HV2060MG specifications](https://servodatabase.com/servo/jx-servo/pdi-hv2060mg) | Needs hardware verification |
| NEMA17-04 / 42SHD0217-24B | Three units appear in the arm BOM; firmware assigns one to upper-arm rotation and one to the elbow. | [Electrokit NEMA17 datasheet](https://www.electrokit.com/upload/quick/ea/2d/5b25_41032662-tds.pdf) | BOM confirmed; mapping incomplete |
| NEMA23-03 / 57SHD4934-34B | Two units appear in the arm BOM but are not assigned by the final shoulder, elbow, or hand firmware readmes. | [Electrokit NEMA23 datasheet](https://www.electrokit.com/upload/quick/e6/0c/2f07_41032667-tds.pdf) | BOM confirmed; mapping unknown |
| Pololu DRV8825 carrier | Stepper-motor driver described for the NEMA 17 motors; current limit and installed carrier revision require verification. | [Pololu DRV8825 carrier](https://www.pololu.com/product/2133) | Needs verification |
| Hitec HS-422 | Three units appear in the hand BOM; the legacy hand firmware names this model for finger motion. | [Electrokit HS-422 datasheet](https://www.electrokit.com/upload/product/41002/41002565/hs422.pdf) | Needs hardware verification |
| Whadda WPK601 | Wrist/lower-arm rotation servo model named by the legacy hand-motor documentation. | [Velleman/Whadda WPK601](https://www.velleman.eu/products/view/270-robot-digital-double-shaft-servo-kit-wpk601/?country=be&id=460528&lang=en) | Needs verification |
| ST LSM6DSO32 | Accelerometer and gyroscope model implemented by the legacy ESP32 IMU component. Confirm the markings and orientation of installed sensors. | [STMicroelectronics LSM6DSO32](https://www.st.com/en/mems-and-sensors/lsm6dso32.html) | Needs verification |
| TI SN65HVD232 | 3.3 V CAN transceiver family identified by the legacy CAN and hardware documentation. Confirm fitted part numbers, termination, and board revisions. | [Texas Instruments SN65HVD232](https://www.ti.com/product/SN65HVD232) | Needs verification |

The old repository also refers to potentiometers, pressure sensors, EMG sensors, piezo sensors, and camera hardware. Their exact installed models or current relevance were not clear enough to list as confirmed resources. Add them only after checking the BOM, schematics, source code, and physical hardware.
