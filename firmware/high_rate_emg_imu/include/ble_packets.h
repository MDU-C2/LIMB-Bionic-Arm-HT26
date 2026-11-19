/* Jonas Gerne, Majd */
#pragma once

#include <stdint.h>

#define SYNC_PROTOCOL_VERSION 1U
#define IMU_PACKET_VERSION 1U
#define EMG_PACKET_VERSION 1U
#define BLE_EMG_SAMPLES_PER_PACKET 40U

/* Packed structs keep the BLE wire layout unchanged. */
typedef struct __attribute__((packed)) {
    uint16_t format_flags;
    uint32_t sequence;
    uint64_t imu1_time_us;
    uint64_t imu2_time_us;
    int16_t values[12];
    /*
     * Accelerometer: stored as mg
     * Gyroscope: stored as 0.01 dps
     * Layout:
     * values[0..5]   = IMU1
     * values[6..11]  = IMU2
     * AX AY AZ GX GY GZ
     */

} ble_imu_packet_t;

typedef struct __attribute__((packed)) {
    uint16_t format_flags;
    uint32_t sequence;
    uint64_t first_sample_time_us; // Timestamp of the first sample in this packet.
    uint32_t sample_period_us;     // Time between EMG samples.
    uint16_t sample_count;         // Number of valid samples in samples[].
    uint16_t block_offset;         // Offset of this packet inside the original emg_block_t.
    uint16_t samples[BLE_EMG_SAMPLES_PER_PACKET];
} ble_emg_packet_t;

typedef struct __attribute__((packed)) {
    uint16_t version;
    uint16_t flags;
    uint32_t exchange_id;
    uint64_t host_tx_ns;
} time_sync_request_t;

typedef struct __attribute__((packed)) {
    uint16_t version;
    uint16_t flags;
    uint32_t exchange_id;
    uint64_t host_tx_ns;
    uint64_t device_rx_us;
    uint64_t device_tx_us;
} time_sync_response_t;

_Static_assert(sizeof(ble_imu_packet_t) == 46, "IMU packet wire size changed");
_Static_assert(sizeof(ble_emg_packet_t) == 102, "EMG packet wire size changed");
_Static_assert(sizeof(time_sync_request_t) == 16, "time-sync request wire size changed");
_Static_assert(sizeof(time_sync_response_t) == 32, "time-sync response wire size changed");
