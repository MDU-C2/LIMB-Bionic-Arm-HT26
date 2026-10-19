#pragma once

#include "esp_err.h"
#include "imu.h"
#include "emg.h"


esp_err_t app_ble_init(void);

esp_err_t ble_send_imu_frame(const imu_frame_t *frame);

esp_err_t ble_send_emg_block(const emg_block_t *block);