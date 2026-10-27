# Bluetooth sensor time synchronization

The dual-IMU/EMG ESP32 uses the LIMB-HT25 service and sensor UUIDs so the existing
Bleak tools continue to work. The firmware keeps the useful HT25 choices—ESP-IDF
NimBLE, compact binary notifications, acquisition sequence numbers, and one
notification per IMU sample—but fixes the old recorder's timing limitation.
HT25 assigned `time.time()` when a notification reached the computer, so radio
and operating-system scheduling jitter appeared as sensor timing jitter.

## Shared time domain

Each IMU read carries its own `device_time_us`; each EMG packet carries its
estimated first sample time and nominal sample period. IMU times are taken
before register reads. EMG time is anchored at the first ADC block read and then
advanced by retained sample count at the configured rate. These are firmware
timestamps, not independent hardware measurements of every ADC conversion.
Clock synchronization cannot detect ADC buffer loss or prove the ADC rate.
On connection, the host performs repeated four-timestamp exchanges:

```text
host t0  ->  device receives t1, sends t2  ->  host receives t3
```

The host removes device processing time from the round trip, prefers the
lowest-latency exchanges, and fits an affine mapping rather than a fixed offset:

```text
host_monotonic_ns = offset + rate * device_time_ns
```

This corrects normal ESP32 oscillator drift during longer trials. Each BLE node
has an independent mapping, but all results use the computer's
`time.monotonic_ns()` clock. OAK-D pose/observation frames now save that same
absolute `host_monotonic_ns`, so camera and sensor rows can be compared even
when notifications arrive late or in bursts. All camera recorders timestamp host frame receipt before tracking.
Neither is the exposure time. This shared clock does not by itself establish
precise physical camera/sensor alignment.

Cup RGB and depth files additionally retain OAK `device_time_ns`. These times
share the camera's device clock and help match RGB to depth; they are not ESP32
times or host-aligned timestamps. A saved host receipt time accompanies each
frame. Tracking uses the closest available depth frame within 20 ms and records
the difference. Keep this matching uncertainty separate from BLE clock-fit error.

## Recorded fields

Current `imu.csv` and `emg.csv` files retain the notification arrival field
`host_monotonic_ns` for diagnostics and add `synced_host_monotonic_ns` for data
alignment. `packets.jsonl` preserves raw payloads and both timestamps. The
session `meta.json` contains exchange count, clock rate/drift, minimum round
trip, and estimated uncertainty.

New CSV rows also retain `connection_generation`. Each queued raw packet keeps
the immutable clock fit available at receipt; a later fit does not rewrite its
mapped time. Reconnects start a new clock epoch and invalidate live calibration.
A large backwards device clock after stale input is treated as a possible reset.
Version 1 has no boot identifier, so this inference requires reconnecting and
calibrating again rather than pretending the old clock fit is still valid.

`packet_health` in `meta.json` records received packets, sequence discontinuities,
estimated missing notifications, and decode errors for each stream. Forward
sequence gaps estimate missing notifications; duplicates, reordered packets and
device resets are counted as discontinuities rather than enormous packet losses.
Sequence tracking handles the 32-bit counter wrapping. No application checksum
or automatic retransmission is provided.

Health also separates duplicate/reordered packets and EMG sample-time gaps.
Continuous notification sequences do not prove continuous sample times. Overlapping
EMG times remain in raw files but are excluded from live activation processing.
Bounded raw-writer queues report overruns and make affected captures incomplete.

If a legacy HT25 peripheral lacks the time-sync characteristic, recording still
works and the synchronized field stays empty. The arrival timestamp remains
available, making the fallback explicit instead of silently claiming hardware
synchronization.

## UUIDs and wire protocol

| Purpose | UUID |
| --- | --- |
| Sensor service | `23011525-1212-efde-1523-785feabcd122` |
| EMG notification | `24011525-1212-efde-1523-785feabcd122` |
| IMU notification | `25011525-1212-efde-1523-785feabcd122` |
| Time-sync write/notify | `27011525-1212-efde-1523-785feabcd122` |

All integer fields are little-endian. A sync request is `<uint16 version,
uint16 flags, uint32 exchange_id, uint64 host_tx_ns>`. The response echoes those
identifiers and appends `<uint64 device_rx_us, uint64 device_tx_us>`. Version is
currently `1`.

The maintained firmware uses a 46-byte IMU notification containing a version
byte, connection mask, sequence, two 64-bit sample times, and twelve signed
16-bit values. Mask bit 0 is IMU1 (`0x6B`) and bit 1 is IMU2 (`0x6A`); absent
roles are omitted by the recorder. Acceleration values are mg and angular
velocity values are 0.01 dps on the wire. EMG notifications contain a 22-byte
header plus 1–40 unsigned 16-bit ADC readings; the header includes sequence,
first sample time, sample period, count, and offset in the ADC read block.
Each BLE notification has its own sequence number so missing packets can be
detected. The recorder saves each decoded EMG sample with its individual time.

Unsupported versions and malformed current packets are rejected. Timestamped
inherited layouts remain separate decoder rules. Headerless EMG requires known
firmware provenance: choose `--emg-format legacy-fixed` for forty samples followed
by a sequence, or `--emg-format legacy-prefix` for a sequence followed by samples.
For example:

```powershell
python -m aurora.recording.record_ble_sensors --emg-format legacy-fixed
```

Automatic decoding cannot distinguish a truncated current packet from an 84-byte
headerless packet reliably.

Current BLE gyros use signed 16-bit values at 0.01 degrees/s per count. Their
wire range is -327.68 to +327.67 degrees/s. A value at either endpoint is retained
with `gyro_saturated=1` in IMU CSV and counted in packet health. It is not used
for gyro integration; live control pauses and requires alignment again. The
earlier packed timestamped layout has a smaller range because it uses mdps.
The wire format and sensor configuration are unchanged.
