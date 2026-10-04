#include <stdio.h>
#include <stdint.h>

#include "config.h"
#include "emg.h"
#include "imu.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// ============================================================
// PRIVATE CONFIGURATION
// ============================================================

static const char *TAG = "MAIN";


// ============================================================
// EMG -> USB
// ============================================================

static void run_emg_usb(void)
{
    printf("\n");
    printf("Starting EMG -> USB test\n");
    printf("EMG sample rate: %d Hz\n", EMG_SAMPLE_RATE_HZ);

    printf(
        "timestamp_us,emg_raw\n"
    );


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


        // ====================================================
        // SECONDARY IMU - address 0x6A
        // ====================================================

        imu_change_sensor_addr(
            IMU_SECONDARY_ADDR
        );


        int64_t secondary_time_us =
            esp_timer_get_time()
            - start_time_us;


        esp_err_t secondary_err =
            imu_read_data(
                &raw_secondary
            );


        // ====================================================
        // PRIMARY IMU - address 0x6B
        // ====================================================

        imu_change_sensor_addr(
            IMU_PRIMARY_ADDR
        );


        int64_t primary_time_us =
            esp_timer_get_time()
            - start_time_us;


        esp_err_t primary_err =
            imu_read_data(
                &raw_primary
            );


        // ====================================================
        // CONVERT + PRINT SECONDARY
        // ====================================================

        if (secondary_err == ESP_OK)
        {
            ImuData secondary =
                imu_to_mg_and_mdps(
                    raw_secondary
                );


            printf(
                "IMU2,%lld,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f\n",

                (long long)secondary_time_us,

                secondary.accel.x,
                secondary.accel.y,
                secondary.accel.z,

                secondary.gyro.pitch,
                secondary.gyro.roll,
                secondary.gyro.yaw
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


        // ====================================================
        // CONVERT + PRINT PRIMARY
        // ====================================================

        if (primary_err == ESP_OK)
        {
            ImuData primary =
                imu_to_mg_and_mdps(
                    raw_primary
                );


            printf(
                "IMU1,%lld,"
                "%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f\n",

                (long long)primary_time_us,

                primary.accel.x,
                primary.accel.y,
                primary.accel.z,

                primary.gyro.pitch,
                primary.gyro.roll,
                primary.gyro.yaw
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


        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(
                IMU_SAMPLE_PERIOD_MS
            )
        );
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
            ESP_LOGW(
                TAG,
                "Combined EMG + IMU USB mode "
                "not implemented yet"
            );

            while (true)
            {
                vTaskDelay(
                    pdMS_TO_TICKS(1000)
                );
            }
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