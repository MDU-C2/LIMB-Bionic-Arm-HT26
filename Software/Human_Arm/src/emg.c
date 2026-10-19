#include "emg.h"
#include "config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "esp_log.h"

#include "soc/soc_caps.h"


// ============================================================
// PRIVATE CONFIGURATION
// ============================================================

static const char *TAG = "EMG";
static adc_continuous_handle_t adc_handle = NULL;   // ESP-IDF ADC handle
static bool emg_initialized = false;                // True after successful initialization
static uint32_t adc_hardware_sample_rate = 0;       // Actual hardware ADC sampling frequency
static uint32_t adc_decimation = 1;                 // Number of hardware samples between each retained EMG sample
static uint32_t adc_decimation_counter = 0;         // Keeps track of samples during decimation
#define ADC_READ_BUFFER_SIZE 1024                   // Temporary buffer used when reading data from the ADC driver
static uint8_t adc_read_buffer[ADC_READ_BUFFER_SIZE];

// ============================================================
// EMG INITIALIZATION
// ============================================================

esp_err_t emg_init(void)
{
    // Prevent double initialization
    if (emg_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // --------------------------------------------------------
    // Determine ADC hardware sampling frequency
    //
    // ESP32 continuous ADC may require a minimum hardware
    // sampling frequency. If our requested EMG rate is lower,
    // we run the ADC faster and later decimate the samples.
    // --------------------------------------------------------

    adc_hardware_sample_rate = EMG_SAMPLE_RATE_HZ;

    if (adc_hardware_sample_rate < SOC_ADC_SAMPLE_FREQ_THRES_LOW) {

        uint32_t multiplier =
            (SOC_ADC_SAMPLE_FREQ_THRES_LOW +
             EMG_SAMPLE_RATE_HZ - 1)
            / EMG_SAMPLE_RATE_HZ;

        adc_hardware_sample_rate =
            EMG_SAMPLE_RATE_HZ * multiplier;
    }

    // Number of hardware samples per retained EMG sample
    adc_decimation =
        adc_hardware_sample_rate / EMG_SAMPLE_RATE_HZ;

    if (adc_decimation == 0) {
        adc_decimation = 1;
    }

    ESP_LOGI(
        TAG,
        "Requested EMG sample rate: %u Hz",
        (unsigned int)EMG_SAMPLE_RATE_HZ
    );

    ESP_LOGI(
        TAG,
        "ADC hardware sample rate: %u Hz",
        (unsigned int)adc_hardware_sample_rate
    );

    ESP_LOGI(
        TAG,
        "ADC decimation factor: %u",
        (unsigned int)adc_decimation
    );


    // --------------------------------------------------------
    // Create continuous ADC handle
    // --------------------------------------------------------

    uint32_t buffer_size =
        SOC_ADC_DIGI_RESULT_BYTES *
        adc_hardware_sample_rate *
        EMG_BUFFER_MS /
        1000;

    // Make sure the buffer is not zero
    if (buffer_size < 256) {
        buffer_size = 256;
    }

    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = buffer_size,
        .conv_frame_size = 256,
    };

    esp_err_t err = adc_continuous_new_handle(
        &handle_config,
        &adc_handle
    );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to create ADC handle: %s",
            esp_err_to_name(err)
        );

        adc_handle = NULL;

        return err;
    }


    // --------------------------------------------------------
    // Configure EMG ADC channel
    // --------------------------------------------------------

    adc_digi_pattern_config_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .channel = EMG_ADC_CHANNEL,
        .unit = EMG_ADC_UNIT,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };


    adc_continuous_config_t adc_config = {
        .pattern_num = 1,
        .adc_pattern = &channel_config,

        .sample_freq_hz =
            adc_hardware_sample_rate,

        .conv_mode =
            ADC_CONV_SINGLE_UNIT_1,

        .format =
            ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };


    err = adc_continuous_config(
        adc_handle,
        &adc_config
    );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to configure ADC: %s",
            esp_err_to_name(err)
        );

        adc_continuous_deinit(adc_handle);

        adc_handle = NULL;

        return err;
    }


    // --------------------------------------------------------
    // Start continuous ADC acquisition
    // --------------------------------------------------------

    err = adc_continuous_start(adc_handle);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to start ADC: %s",
            esp_err_to_name(err)
        );

        adc_continuous_deinit(adc_handle);

        adc_handle = NULL;

        return err;
    }


    adc_decimation_counter = 0;
    emg_initialized = true;

    ESP_LOGI(TAG, "EMG ADC started successfully");

    return ESP_OK;
}


// ============================================================
// READ EMG SAMPLES
// ============================================================

esp_err_t emg_read_samples(
    uint16_t *samples,
    size_t capacity,
    size_t *sample_count,
    uint32_t timeout_ms)
{
    // --------------------------------------------------------
    // Validate arguments
    // --------------------------------------------------------

    if (!emg_initialized ||
        samples == NULL ||
        sample_count == NULL ||
        capacity == 0) {

        return ESP_ERR_INVALID_ARG;
    }


    *sample_count = 0;


    // --------------------------------------------------------
    // Read a block from the ESP-IDF ADC driver
    // --------------------------------------------------------

    uint32_t bytes_read = 0;

    esp_err_t err = adc_continuous_read(
        adc_handle,
        adc_read_buffer,
        sizeof(adc_read_buffer),
        &bytes_read,
        timeout_ms
    );

    if (err != ESP_OK) {
        return err;
    }


    // --------------------------------------------------------
    // Interpret ADC conversion results
    // --------------------------------------------------------

    size_t conversion_count =
        bytes_read /
        sizeof(adc_digi_output_data_t);


    adc_digi_output_data_t *results =
        (adc_digi_output_data_t *)adc_read_buffer;


    for (size_t i = 0;
         i < conversion_count;
         ++i) {

        // ----------------------------------------------------
        // Ignore samples from other ADC channels
        // ----------------------------------------------------

        if (results[i].type2.channel != EMG_ADC_CHANNEL) {
            continue;
        }


        // ----------------------------------------------------
        // Downsample if the ADC hardware runs faster than
        // the requested EMG sampling frequency
        // ----------------------------------------------------

        adc_decimation_counter++;

        if (adc_decimation_counter < adc_decimation) {
            continue;
        }

        adc_decimation_counter = 0;


        // ----------------------------------------------------
        // Stop if caller's output buffer is full
        // ----------------------------------------------------

        if (*sample_count >= capacity) {
            break;
        }


        // ----------------------------------------------------
        // Store raw ADC value
        // ----------------------------------------------------

        samples[*sample_count] =
            results[i].type2.data;

        (*sample_count)++;
    }


    return ESP_OK;
}


// ============================================================
// EMG DEINITIALIZATION
// ============================================================

esp_err_t emg_deinit(void)
{
    // Already stopped
    if (!emg_initialized) {
        return ESP_OK;
    }


    esp_err_t stop_err =
        adc_continuous_stop(adc_handle);


    esp_err_t deinit_err =
        adc_continuous_deinit(adc_handle);


    // Reset internal state
    adc_handle = NULL;

    emg_initialized = false;

    adc_hardware_sample_rate = 0;

    adc_decimation = 1;

    adc_decimation_counter = 0;


    // Return first relevant error
    if (stop_err != ESP_OK) {
        return stop_err;
    }

    return deinit_err;
}