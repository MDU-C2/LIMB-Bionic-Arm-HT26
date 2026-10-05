#include <stdio.h>
#include <stdint.h>

#include "config.h"
#include "emg.h"
#include "imu.h"
#include "ble.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// ============================================================
// PRIVATE CONFIGURATION
// ============================================================

static const char *TAG = "MAIN";

// Average function
void expAvgR3(float *x,float *y,float *z,float readingX,float readingY,float readingZ)
{
    const float alpha = 0.05f;
    *x = alpha * readingX + (1.0f - alpha) * (*x);
    *y = alpha * readingY + (1.0f - alpha) * (*y);
    *z = alpha * readingZ + (1.0f - alpha) * (*z);
}


// ============================================================
// EMG -> USB
// ============================================================

static void run_emg_usb(void)
{
    printf("\n");
    printf("Starting EMG -> USB test\n");
    printf("EMG sample rate: %d Hz\n", EMG_SAMPLE_RATE_HZ);
    printf("timestamp_us,emg_raw\n");

    uint16_t samples[EMG_BLOCK_SIZE];
    uint64_t sample_index = 0;

    while (true)
    {
        size_t sample_count = 0;
        esp_err_t err =
            emg_read_samples(
                samples,
                EMG_BLOCK_SIZE,
                &sample_count,
                EMG_READ_TIMEOUT_MS
            );


        if (err == ESP_ERR_TIMEOUT)
        {
            continue;
        }


        if (err != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "EMG read error: %s",
                esp_err_to_name(err)
            );

            vTaskDelay(
                pdMS_TO_TICKS(10)
            );

            continue;
        }


        for (size_t i = 0;
             i < sample_count;
             ++i)
        {
            uint64_t timestamp_us =
                (sample_index * 1000000ULL) /
                EMG_SAMPLE_RATE_HZ;


            if ((sample_index %
                 EMG_USB_PRINT_DIVIDER) == 0)
            {
                printf(
                    "%llu,%u\n",
                    (unsigned long long)timestamp_us,
                    samples[i]
                );
            }


            sample_index++;
        }


        taskYIELD();
    }
}

// ============================================================
// IMU INITIALIZATION
// ============================================================

static esp_err_t initialize_imus(void)
{
    ImuConfig imu_cfg =
        IMU_CONFIG_DEFAULT();


    // Use our central config.h instead of
    // hard-coded values in main.c.
    imu_cfg.i2c_port =
        IMU_I2C_PORT;

    imu_cfg.sda_pin =
        IMU_SDA_PIN;

    imu_cfg.scl_pin =
        IMU_SCL_PIN;

    imu_cfg.i2c_freq_hz =
        IMU_I2C_FREQ_HZ;

    imu_cfg.sensor_addr =
        IMU_PRIMARY_ADDR;


    // Initialize I2C and primary IMU.
    esp_err_t err =
        imu_init(
            &imu_cfg
        );


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to initialize primary IMU: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    // Switch to second address and configure
    // the second IMU exactly like the
    // original dual-IMU main.c.
    imu_change_sensor_addr(
        IMU_SECONDARY_ADDR
    );


    err =
        secondarySensorConfig();


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to configure secondary IMU: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    ESP_LOGI(
        TAG,
        "Both IMUs initialized"
    );


    return ESP_OK;
}

// ============================================================
// IMU -> USB
// ============================================================

static void run_imu_usb(void)
{
    float gyro1x, gyro1y, gyro1z, gyro2x, gyro2y, gyro2z;
    float accel1x, accel1y, accel1z, accel2x, accel2y, accel2z;
    printf("\n");
    printf("Starting IMU1 + IMU2 -> USB test\n");

    printf(
        "sensor,timestamp_us,"
        "ax,ay,az,"
        "gx,gy,gz\n"
    );


    TickType_t last_wake =
        xTaskGetTickCount();


    int64_t start_time_us =
        esp_timer_get_time();


    while (true)
    {
        ImuRawData raw_primary = {0};
        ImuRawData raw_secondary = {0};

        // SECONDARY IMU - address 0x6A
        imu_change_sensor_addr(IMU_SECONDARY_ADDR);
        int64_t secondary_time_us = esp_timer_get_time() - start_time_us;
        esp_err_t secondary_err = imu_read_data(&raw_secondary);
       

        // PRIMARY IMU - address 0x6B
        imu_change_sensor_addr(IMU_PRIMARY_ADDR);
        int64_t primary_time_us = esp_timer_get_time() - start_time_us;
        esp_err_t primary_err = imu_read_data(&raw_primary);


        // CONVERT + PRINT SECONDARY
        if (secondary_err == ESP_OK)
        {
            
            ImuData secondary = imu_to_mg_and_mdps(raw_secondary);
            expAvgR3(&accel2x, &accel2y, &accel2z, secondary.accel.x, secondary.accel.y, secondary.accel.z);
            expAvgR3(&gyro2x, &gyro2y, &gyro2z, secondary.gyro.pitch, secondary.gyro.roll, secondary.gyro.yaw);
            printf("IMU2,%lld," "%.3f,%.3f,%.3f," "%.3f,%.3f,%.3f\n",
                (long long)secondary_time_us,
                accel2x, accel2y, accel2z,
                gyro2x, gyro2y, gyro2z
            );
        }
        else
        {
            ESP_LOGW(TAG, "Secondary IMU read failed: %s", esp_err_to_name(secondary_err));
        }

        // CONVERT + PRINT PRIMARY
        if (primary_err == ESP_OK)
        {
            ImuData primary = imu_to_mg_and_mdps(raw_primary);
            expAvgR3(&accel1x, &accel1y, &accel1z, primary.accel.x, primary.accel.y, primary.accel.z);
            expAvgR3(&gyro1x, &gyro1y, &gyro1z, primary.gyro.pitch, primary.gyro.roll, primary.gyro.yaw);
            printf("IMU1,%lld," "%.3f,%.3f,%.3f," "%.3f,%.3f,%.3f\n",
                (long long)primary_time_us,
                accel1x, accel1y, accel1z,
                gyro1x, gyro1y, gyro1z
            );
        }
        else
        {
            ESP_LOGW( TAG, "Primary IMU read failed: %s", esp_err_to_name(primary_err));
        }
        

        /*vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(
                IMU_SAMPLE_PERIOD_MS
            )
        );*/
    }
}

// ============================================================
// EMG + IMU -> USB
// ============================================================

static void run_emg_imu_usb(void)
{

    float gyro1x = 0.0f;
    float gyro1y = 0.0f;
    float gyro1z = 0.0f;

    float gyro2x = 0.0f;
    float gyro2y = 0.0f;
    float gyro2z = 0.0f;

    float accel1x = 0.0f;
    float accel1y = 0.0f;
    float accel1z = 0.0f;

    float accel2x = 0.0f;
    float accel2y = 0.0f;
    float accel2z = 0.0f;

    bool imu1_filter_initialized = false;
    bool imu2_filter_initialized = false;

    printf("\n");
    printf("Starting EMG + IMU -> USB test\n");
    printf("sensor,timestamp_us,data...\n");

    // EMG variables
    uint16_t emg_samples[EMG_BLOCK_SIZE];
    uint64_t emg_sample_index = 0;

    // IMU variables
    ImuRawData raw_primary = {0};
    ImuRawData raw_secondary = {0};

    // Common ESP32 timebase
    const int64_t start_time_us = esp_timer_get_time();
    int64_t next_imu_time_us = start_time_us;

    while (true)
    {
        // EMG
        size_t emg_sample_count = 0;
        esp_err_t emg_err = emg_read_samples(emg_samples, EMG_BLOCK_SIZE, &emg_sample_count, EMG_IMU_READ_TIMEOUT_MS);
        if (emg_err == ESP_OK)
        {
            const uint64_t sample_period_us = 1000000ULL / EMG_SAMPLE_RATE_HZ;
            int64_t block_read_time_us = esp_timer_get_time() - start_time_us;
            uint64_t block_duration_us = 0;

            if (emg_sample_count > 0)
            {
                block_duration_us = (emg_sample_count - 1) * sample_period_us;
            }

            int64_t first_sample_time_us = block_read_time_us - (int64_t)block_duration_us;

            for (size_t i = 0; i < emg_sample_count; i++)
            {
                int64_t timestamp_us = first_sample_time_us + (int64_t)(i * sample_period_us);

                if ((emg_sample_index % EMG_USB_PRINT_DIVIDER) == 0)
                {
                    printf(
                        "EMG,%lld,%u\n",
                        (long long)timestamp_us,
                        emg_samples[i]
                    );
                }
                emg_sample_index++;
            }
        }
        else if (emg_err != ESP_ERR_TIMEOUT)
        {
            ESP_LOGW(
                TAG,
                "EMG read failed: %s",
                esp_err_to_name(emg_err)
            );
        }

        // Check if it is time to read the IMUs
        int64_t current_time_us = esp_timer_get_time();
        if (current_time_us >= next_imu_time_us)
        {
            // Secondary IMU - 0x6A
            imu_change_sensor_addr(IMU_SECONDARY_ADDR);
            int64_t secondary_timestamp_us = esp_timer_get_time() - start_time_us;
            esp_err_t secondary_err = imu_read_data(&raw_secondary);

            // Primary IMU - 0x6B
            imu_change_sensor_addr(IMU_PRIMARY_ADDR);
            int64_t primary_timestamp_us = esp_timer_get_time() - start_time_us;
            esp_err_t primary_err = imu_read_data(&raw_primary);

            // Secondary IMU output
            if (secondary_err == ESP_OK)
            {
                ImuData secondary = imu_to_mg_and_mdps(raw_secondary);
                if (!imu2_filter_initialized)
                {
                    accel2x = secondary.accel.x;
                    accel2y = secondary.accel.y;
                    accel2z = secondary.accel.z;

                    gyro2x = secondary.gyro.pitch;
                    gyro2y = secondary.gyro.roll;
                    gyro2z = secondary.gyro.yaw;

                    imu2_filter_initialized = true;
                }
                else
                {
                    expAvgR3(&accel2x,&accel2y,&accel2z,secondary.accel.x,secondary.accel.y,secondary.accel.z);
                    expAvgR3(&gyro2x,&gyro2y,&gyro2z,secondary.gyro.pitch,secondary.gyro.roll,secondary.gyro.yaw);
                }

                printf(
                    "IMU2,%lld,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f\n",
                    (long long)secondary_timestamp_us,
                    accel2x,
                    accel2y,
                    accel2z,
                    gyro2x,
                    gyro2y,
                    gyro2z
                );
            }
            else
            {
                ESP_LOGW(
                    TAG,
                    "Secondary IMU read failed: %s",
                    esp_err_to_name(secondary_err)
                );
            }



            // Primary IMU output
            if (primary_err == ESP_OK)
            {
                ImuData primary = imu_to_mg_and_mdps(raw_primary);

                if (!imu1_filter_initialized)
                {
                    accel1x = primary.accel.x;
                    accel1y = primary.accel.y;
                    accel1z = primary.accel.z;
                    gyro1x = primary.gyro.pitch;
                    gyro1y = primary.gyro.roll;
                    gyro1z = primary.gyro.yaw;
                    imu1_filter_initialized = true;
                }
                else
                {
                    expAvgR3(&accel1x,&accel1y,&accel1z,primary.accel.x,primary.accel.y,primary.accel.z);
                    expAvgR3(&gyro1x,&gyro1y,&gyro1z,primary.gyro.pitch,primary.gyro.roll,primary.gyro.yaw);
                }

                printf(
                    "IMU1,%lld,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f\n",

                    (long long)primary_timestamp_us,

                    accel1x,
                    accel1y,
                    accel1z,

                    gyro1x,
                    gyro1y,
                    gyro1z
                );
            }
            else
            {
                ESP_LOGW(
                    TAG,
                    "Primary IMU read failed: %s",
                    esp_err_to_name(primary_err)
                );
            }

            // Schedule next IMU sample
            next_imu_time_us += IMU_SAMPLE_PERIOD_MS * 1000LL;
            /*
             * If something delayed us significantly,
             * prevent the program from trying to catch up
             * by reading many IMU samples immediately.
             */
            current_time_us = esp_timer_get_time();

            if (next_imu_time_us < current_time_us)
            {
                next_imu_time_us =
                    current_time_us
                    + IMU_SAMPLE_PERIOD_MS * 1000LL;
            }
        }


        taskYIELD();
    }
}

// ============================================================
// MAIN
// ============================================================

void app_main(void)
{
    // ========================================================
    // INITIALIZE SELECTED SENSORS
    // ========================================================

    if (SENSOR_MODE == SENSOR_MODE_EMG ||
        SENSOR_MODE == SENSOR_MODE_EMG_IMU)
    {
        ESP_LOGI(
            TAG,
            "Initializing EMG"
        );


        ESP_ERROR_CHECK(
            emg_init()
        );
    }


    if (SENSOR_MODE == SENSOR_MODE_IMU ||
        SENSOR_MODE == SENSOR_MODE_EMG_IMU)
    {
        ESP_LOGI(
            TAG,
            "Initializing IMUs"
        );


        ESP_ERROR_CHECK(
            initialize_imus()
        );
    }


    // ========================================================
    // USB TEST MODES
    // ========================================================

    if (TRANSPORT_MODE == TRANSPORT_USB)
    {
        if (SENSOR_MODE == SENSOR_MODE_EMG)
        {
            run_emg_usb();
        }


        if (SENSOR_MODE == SENSOR_MODE_IMU)
        {
            run_imu_usb();
        }


        if (SENSOR_MODE == SENSOR_MODE_EMG_IMU)
        {
            run_emg_imu_usb();
        }
    }


    // ========================================================
    // BLE
    // ========================================================

    if (TRANSPORT_MODE == TRANSPORT_BLE)
    {
        ESP_LOGW(
            TAG,
            "BLE transport not implemented yet"
        );


        while (true)
        {
            vTaskDelay(
                pdMS_TO_TICKS(1000)
            );
        }
    }
}