# LIMB – Bionic Arm

Project repository for the **DVA490 / DVA474 – Projektkurs i Robotik** course at Mälardalen University (MDU), autumn semester 2026 (HT2026).

Course center: [Projects @ Collaborative Center (C2) MDU](https://github.com/MDU-C2)

## Encoder-free manual motor test

This branch is a temporary bench-test configuration for all four arm motors.
It deliberately does not use physical encoder feedback:

- `robot_shoulder_module`: shoulder up/down, shoulder left/right and upper-arm rotation.
- `robot_elbow_module`: bicep/elbow rotation.
- `motor_control.py`: manual CAN GUI for selecting one or more motors.

Both controllers publish an estimated angle so the GUI can validate that CAN
communication is alive before it sends a movement command. The estimate starts
at `0 deg` every time each ESP32 boots. It is not a measurement of the real arm.

### Test limits

- Target range: `-10` to `+10 deg` relative to the boot position.
- Maximum change per GUI command: `5 deg`.
- Maximum speed: `1 deg/s`.
- The GUI sends STOP automatically after 7 seconds.
- A new explicit START command rearms an axis stopped by CAN STOP.

Place every joint in a safe unloaded middle position before boot and treat that
position as zero. Use an independent physical power cutoff. This firmware must
not be used with a person attached to the arm.

### CAN protocol

| Axis | Actuation | STOP | Estimated angle |
| --- | ---: | ---: | ---: |
| Bicep / elbow | `0x240` | `0x140` | `0x4A0` |
| Shoulder up/down | `0x220` | `0x120` | `0x4A2` |
| Shoulder left/right | `0x221` | `0x121` | `0x4A3` |
| Upper-arm rotation | `0x222` | `0x122` | `0x4A1` |

Actuation contains two little-endian 32-bit floats: absolute test angle in
degrees followed by speed in degrees per second. Angle feedback contains one
little-endian 32-bit float. CAN runs at 1 Mbit/s.

### GPIO mapping used by this test firmware

Verify these signals against the physical PCB before powering the motors:

| Controller / axis | STEP | DIR | CAN TX | CAN RX |
| --- | ---: | ---: | ---: | ---: |
| Elbow ESP32-C3 | GPIO 7 | GPIO 6 | GPIO 1 | GPIO 0 |
| Shoulder up/down | GPIO 19 | GPIO 18 | GPIO 1 | GPIO 3 |
| Shoulder left/right | GPIO 0 | GPIO 4 | GPIO 1 | GPIO 3 |
| Upper-arm rotation | GPIO 15 | GPIO 2 | GPIO 1 | GPIO 3 |

### Build firmware

From PowerShell with PlatformIO installed:

```powershell
cd robot_elbow_module
pio run
cd ..\robot_shoulder_module
pio run
```

Successful builds create:

- `robot_elbow_module/.pio/build/elbow_esp32c3/firmware.bin`
- `robot_shoulder_module/.pio/build/shoulder_esp32/firmware.bin`

Building does not flash either controller. Flash only the matching firmware to
each board after checking the GPIO mapping above.

### GUI on Orin

Run in demo mode first:

```sh
python3 motor_control.py
```

After both ESP32 programs are built, flashed and connected to `can0`, run:

```sh
python3 motor_control.py --live --verified elbow updown leftright upper
```
