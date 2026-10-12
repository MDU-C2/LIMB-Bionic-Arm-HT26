# Hardware validation — 10 October 2026

Tested on the connected ESP32-C3 at COM5 and OAK-D Lite on the Windows host.
This verifies this setup; AGX ORIN delivery has not been tested.

## Changes verified

The installed firmware initially sent USB sensor output alongside BLE. Short
captures lost roughly 4–11% of notifications. Selecting BLE-only output and
suppressing verbose NimBLE notification logs did not resolve the loss: the
firmware reported `BLE_HS_ENOMEM` while sending notifications.

The maintained firmware now also restores the link configuration from
LIMB-HT25's `src/esp32/components/limb_ble_periph/src/gap.c`: a data-length
extension request for 251 bytes / 2120 µs, a 2M PHY request, and a 10 ms
connection-interval request. These are requests; the central decides which
settings are accepted. The initial Windows connection interval was 60 ms.
The observed ATT MTU was 256. Packet formats and sample rates are unchanged.

Normal recording now displays estimated missing notifications and saves
`packet_health` in `meta.json`. Disconnects stop recording and preserve files;
BLE connection errors print a concise message. The scanner is available as:

```powershell
python scripts/check_ble.py --name LIMBServer
```

## Results

After flashing the updated firmware, the normal recorder ran for 30 seconds
alongside camera recording and pose/hand processing.

| Stream | Received | Result |
| --- | --- | --- |
| EMG | 120,000 samples in 3,900 notifications | 4,000 Hz; every saved sample interval was 250 µs |
| IMU1 | 3,003 samples | Approximately 100 Hz; monotonically increasing sample times |
| IMU2 | 3,003 samples | Approximately 100 Hz; monotonically increasing sample times |
| BLE packet checks | Both streams | Zero sequence gaps, estimated missing notifications or decode errors |
| Clock sync | 11 valid exchanges | Available; minimum round trip 14.99 ms, estimated uncertainty 7.495 ms |
| Camera | 581 processed frames over 30.015 seconds | RGB recording worked; no person was visible |

The camera saved 750 video frames, including duplicates used to retain
wall-clock playback duration. Processing ran at about 19.4 frames/s; the video
frame count is not the number of independent camera observations.

A further 20-second test used the real BLE reader and camera with the interactive
simulator's kinematic controls and PyBullet DIRECT physics. Live feedback started,
IMU angles and EMG activation reached control state, and no sequence gaps or
decode errors were recorded. Desktop widgets were represented by test widgets;
camera acquisition, sensor processing and physics were real.

The existing HT25-based EMG processor also accepted the full recording:
20–450 Hz band-pass, 50 Hz notch, and 199 RMS/MAV windows produced finite output.
This validates processing, not the biological signal or amplifier.

A follow-up check exercised the live graph window against the same BLE reader
used for control. All five charts drew received data; EMG auto-scale and reopening
the window also worked. The check received 305 IMU and 394 EMG notifications
with zero sequence gaps, missing notifications or decode errors. The chart
buffers stayed bounded at 800 EMG samples and 300 samples per IMU.

The full launcher GUI was also checked with connected hardware. Automatic BLE
startup, read-only device fields and the in-app recording guide worked. Starting
a three-second BLE recording released the preview connection first; it recorded
301 IMU and 389 EMG notifications with zero packet-health errors. The preview
then reconnected automatically. Those temporary smoke-test files were removed.

## Remaining checks

The live-control audit also compared the final HT25 actuator firmware and
DMP-arm's camera kinematics. Offline checks reproduce an active resting EMG
stream alongside camera hand opening/closing. The GUI now defaults to Camera
grip, with explicit EMG and Auto selection, so incoming samples from unattached
electrodes do not suppress camera grip. Live grip respects the old finger-servo
speeds. Real headless PyBullet checks verify open/closed finger poses against
the loaded URDF limits, manual wrist/upper-arm travel and camera axial priority.
The rotation extension adds IMU upper-arm twist, camera correction of the gyro
state and relative forearm twist with elbow-bend compensation. It uses explicit
signed mounting axes and a half-second stationary gyro baseline. Real firmware
packet decoding through live input to PyBullet was checked for both rotation
joints; new tests cover common movement at a bent elbow, isolated elbow bending,
rest bias, axis signs, camera gaps, calibration and immediate reversal at limits.
The earlier 30-second recording was replayed through the new estimator: 3,003
packets, 2,954 with calibrated twist, and zero unwanted upper-arm or wrist commands.
All 150 software tests passed, and the actual Tk grip/axis selectors and launcher
arguments were checked. Sensor boards were loose for this rotation work; mounted
human movement has not been checked. Follow the mounting checks in SIMULATION.md.
These are software checks; they do not establish human motion or muscle accuracy.

- Raw EMG ranged from 0 to 72 ADC counts, with a mean of 8.35. The electrodes
  were not attached during these checks. A rest/clench trial and analogue amplifier
  measurements are still needed, especially after the resistor changes.
- No person was visible, so arm/hand tracking accuracy was not validated.
  Stereo depth was disabled.
- Camera timestamps are currently recorded after tracking. Clock mapping and
  timestamps alone do not establish physical acquisition alignment.
- A short successful trial does not establish long-run loss-free delivery.
  Check `packet_health` for each dataset session; there is no automatic replay
  or application checksum.

## Saved evidence and rollback

- [BLE recording metadata](../outputs/recordings/20261010_131157_128651_hardware_validation_ht25_link_ble/meta.json)
- [Sample timing and ADC checks](../outputs/recordings/20261010_131157_128651_hardware_validation_ht25_link_ble/validation.json)
- [EMG processing check](../outputs/recordings/20261010_131157_128651_hardware_validation_ht25_link_ble/processing_validation.json)
- [Camera recording metadata](../outputs/recordings/20261010_131159_317572_hardware_validation_ht25_link_oak_pose/meta.json)
- [Live simulation check](../outputs/recordings/20261010_131337_360094_hardware_validation_ht25_link_live_simulation/meta.json)

The original bootloader, partition table and application were preserved in
`outputs/firmware_backup/before_ble_validation_20261010_COM5/`. Their MD5 hashes
were verified against the installed flash before replacement. The manifest
contains addresses, lengths and SHA-256 hashes. NVS was not included in this
backup or overwritten during flashing. Full-flash USB reads failed, so this
backup uses cached build images verified against the device instead.

The earlier firmware build/flash and its 125-test software run passed. Raw recordings and
the rollback images are local artifacts; they are not substitutes for the
maintained source files.
