# Bluetooth sensor time synchronization

The dual-IMU/EMG ESP32 uses the LIMB-HT25 service and sensor UUIDs so the existing
Bleak tools continue to work. The firmware keeps the useful HT25 choices—ESP-IDF
NimBLE, compact binary notifications, acquisition sequence numbers, and one
notification per IMU sample—but fixes the old recorder's timing limitation.
HT25 assigned `time.time()` when a notification reached the computer, so radio
and operating-system scheduling jitter appeared as sensor timing jitter.

## Shared time domain

Every packet now carries `device_time_us` from `esp_timer_get_time()` at
acquisition. On connection, the host performs repeated four-timestamp exchanges:

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
absolute `host_monotonic_ns`, so camera and sensor rows can be joined directly
even when notifications arrive late or in bursts.

## Recorded fields

Current `imu.csv` and `emg.csv` files retain the notification arrival field
`host_monotonic_ns` for diagnostics and add `synced_host_monotonic_ns` for data
alignment. `packets.jsonl` preserves raw payloads and both timestamps. The
session `meta.json` contains exchange count, clock rate/drift, minimum round
trip, and estimated uncertainty.

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
