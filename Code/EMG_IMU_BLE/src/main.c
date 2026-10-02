#include <stdio.h>
#include <stdint.h>

#include "config.h"
#include "emg.h"

#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// ============================================================
// EMG + USB TEST
// ============================================================

static void run_emg_usb(void)
{
    printf("\n");
    printf("Starting EMG -> USB test\n");
    printf("EMG sample rate: %d Hz\n",
           EMG_SAMPLE_RATE_HZ);

    printf("\n");
    printf("timestamp_us,emg_raw\n");

    // Buffer used to receive EMG samples from emg.c.
    uint16_t samples[EMG_BLOCK_SIZE];
    // Counts every EMG sample produced by the ADC.
    uint64_t sample_index = 0;

    while (true) {
        size_t sample_count = 0;
        
        // Get available EMG samples
        esp_err_t err =
            emg_read_samples(
                samples,
                EMG_BLOCK_SIZE,
                &sample_count,
                EMG_READ_TIMEOUT_MS
            );

        // Timeout simply means no data was available yet.
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }

        if (err != ESP_OK) {
            printf("EMG read error: %s\n",esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Process received samples
        for (size_t i = 0;i < sample_count;++i) {
            uint64_t timestamp_us = (sample_index * 1000000ULL) / EMG_SAMPLE_RATE_HZ;


            // ------------------------------------------------
            // USB output decimation
            // ------------------------------------------------
            //
            // ADC still samples at full EMG_SAMPLE_RATE_HZ.
            //
            // We only reduce how many values are printed.
            //
            if ((sample_index % EMG_USB_PRINT_DIVIDER) == 0) {

                printf("%llu,%u\n", (unsigned long long)timestamp_us, samples[i]);
            }
            sample_index++;
        }
        // Give FreeRTOS other tasks CPU time.
        taskYIELD();
    }
}

void app_main(void)
{
    // --------------------------------------------------------
    // Initialize selected sensors
    // --------------------------------------------------------

    if (SENSOR_MODE == SENSOR_MODE_EMG || SENSOR_MODE == SENSOR_MODE_EMG_IMU) {
        ESP_ERROR_CHECK(
            emg_init()
        );
    }
    // --------------------------------------------------------
    // Run selected system configuration
    // --------------------------------------------------------

    if (SENSOR_MODE == SENSOR_MODE_EMG && TRANSPORT_MODE == TRANSPORT_USB) {
        run_emg_usb();
    }


    // --------------------------------------------------------
    // Future modes
    // --------------------------------------------------------

    /*
    if (SENSOR_MODE == SENSOR_MODE_IMU &&
        TRANSPORT_MODE == TRANSPORT_USB) {

        run_imu_usb();
    }


    if (SENSOR_MODE == SENSOR_MODE_EMG_IMU &&
        TRANSPORT_MODE == TRANSPORT_USB) {

        run_emg_imu_usb();
    }


    if (TRANSPORT_MODE == TRANSPORT_BLE) {

        run_ble();
    }
    */
}