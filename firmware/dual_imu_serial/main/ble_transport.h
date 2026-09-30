#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* One physical IMU sample in the units used by the desktop application. */
typedef struct {
  bool connected;
  float accel_g[3];
  float gyro_dps[3];
} aurora_ble_imu_sample_t;

/* Start the ESP-IDF NimBLE peripheral and AURORA GATT service. */
esp_err_t ble_transport_init(void);

/* Publish one acquisition-time-stamped pair of role-ordered IMU samples and
 * the EMG ADC sample acquired on the same firmware tick. */
void ble_transport_publish(uint32_t sequence, uint64_t device_time_us,
                           const aurora_ble_imu_sample_t *shoulder,
                           const aurora_ble_imu_sample_t *wrist,
                           bool emg_connected, uint16_t emg_raw);
