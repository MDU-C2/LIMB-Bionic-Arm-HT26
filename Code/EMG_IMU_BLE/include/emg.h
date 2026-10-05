#pragma once

#include <stddef.h>
#include <stdint.h>

#include <config.h>
#include "esp_err.h"

// Initialize continuous ADC acquisition for the EMG sensor.
esp_err_t emg_init(void);

esp_err_t emg_read_samples(
    uint16_t *samples,      // Buffer supplied by the caller
    size_t capacity,        // Maximum number of samples that fit in the buffer
    size_t *sample_count,   // Number of valid samples written to the buffer
    uint32_t timeout_ms     // Maximum time to wait for ADC data
);

// Stop and deinitialize the EMG ADC.
esp_err_t emg_deinit(void);

typedef struct
{
    uint32_t sequence;
    uint64_t first_sample_time_us;
    uint32_t sample_period_us;
    uint16_t sample_count;
    uint16_t samples[EMG_BLOCK_SIZE];
} emg_block_t;