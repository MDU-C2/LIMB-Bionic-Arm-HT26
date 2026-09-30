
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_err.h"

// ==========================================
// Configuration
// ==========================================

#define EMG_CHANNEL ADC_CHANNEL_0  // GPIO4
#define SAMPLE_INTERVAL_MS 10      // ~100 Hz

// ==========================================
// Initial value setup
// ==========================================

int emg_value = 0;
int64_t start_time = 0;

// ==========================================
// Main
// ==========================================

extern "C" void app_main(void)
{
    // Initialize ADC1
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;

    ESP_ERROR_CHECK(adc_oneshot_new_unit(
        &init_config,
        &adc_handle
    ));

    // Configure EMG ADC channel
    adc_oneshot_chan_cfg_t channel_config = {};

    channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    channel_config.atten = ADC_ATTEN_DB_12;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle,
        EMG_CHANNEL,
        &channel_config
    ));

    int emg_value = 0;

    // CSV header
    printf("timestamp_us,emg_raw\n");

    while (true)
    {
        // Read EMG
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, EMG_CHANNEL, &emg_value));
        // Get current timestamp
        int64_t timestamp = esp_timer_get_time();

        // Set first measurement as t = 0
        if (start_time == 0)
        {
            start_time = timestamp;
        }

        // Relative timestamp
        int64_t relative_time = timestamp - start_time;

        // Output: timestamp, ADC value
        printf("%lld,%d\n",(long long)relative_time, emg_value);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
