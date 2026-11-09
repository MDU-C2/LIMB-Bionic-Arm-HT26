/* Jonas Gerne, Cornelia, Majd */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "ble.h"
#include "config.h"
#include "emg.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

static const char *TAG = "SENSORS";

typedef struct {
    uint8_t sda;
    uint8_t scl;
    const char *name;
} wiring_profile_t;

/* Keep the current harness first, then the two previously supported layouts. */
static const wiring_profile_t wiring_profiles[] = {
    {IMU_SDA_PIN, IMU_SCL_PIN, "current harness"},
    {4, 5, "LIMB-HT25"},
    {8, 5, "AURORA prototype"},
};

static bool imu_bus_ready;
static bool imu_configured[2];
static int64_t next_probe_us;

static ImuAddress address_for_role(unsigned role)
{
    return role == 0 ? IMU_PRIMARY_ADDR : IMU_SECONDARY_ADDR;
}

static bool configure_role(unsigned role)
{
    imu_change_sensor_addr(address_for_role(role));
    if (!imu_is_present() || secondarySensorConfig() != ESP_OK) {
        return false;
    }
    imu_configured[role] = true;
    ESP_LOGI(TAG, "IMU%u at 0x%02x ready", role + 1, address_for_role(role));
    return true;
}

static void scan_imu_bus(void)
{
    next_probe_us = esp_timer_get_time() + 1000000;
    if (imu_bus_ready) {
        for (unsigned role = 0; role < 2; ++role) {
            if (!imu_configured[role]) {
                configure_role(role);
            }
        }
        return;
    }

    ImuConfig config = IMU_CONFIG_DEFAULT();
    config.i2c_port = IMU_I2C_PORT;
    config.i2c_freq_hz = IMU_I2C_FREQ_HZ;
    for (unsigned profile = 0; profile < sizeof(wiring_profiles) / sizeof(wiring_profiles[0]);
         ++profile) {
        config.sda_pin = wiring_profiles[profile].sda;
        config.scl_pin = wiring_profiles[profile].scl;
        for (unsigned role = 0; role < 2; ++role) {
            config.sensor_addr = address_for_role(role);
            if (imu_init(&config) == ESP_OK) {
                imu_bus_ready = true;
                imu_configured[role] = true;
                ESP_LOGI(TAG, "IMU bus on SDA=%u SCL=%u (%s)", config.sda_pin, config.scl_pin,
                         wiring_profiles[profile].name);
                configure_role(1 - role);
                return;
            }
        }
    }
    ESP_LOGW(TAG, "No IMU found; retrying while EMG continues");
}

static void smooth_vector(float output[3], const float input[3], bool initialized)
{
    for (unsigned axis = 0; axis < 3; ++axis) {
        output[axis] = initialized ? 0.05f * input[axis] + 0.95f * output[axis] : input[axis];
    }
}

static void read_imu_frame(bool usb, bool ble, uint32_t *sequence)
{
    if (!imu_bus_ready) {
        return;
    }
    static float filtered_accel[2][3];
    static float filtered_gyro[2][3];
    static bool filter_ready[2];
    imu_frame_t frame = {.sequence = (*sequence)++};
    for (unsigned role = 0; role < 2; ++role) {
        if (!imu_configured[role]) {
            continue;
        }
        ImuRawData raw = {0};
        imu_change_sensor_addr(address_for_role(role));
        int64_t timestamp_us = esp_timer_get_time();
        if (imu_read_data(&raw) != ESP_OK) {
            imu_configured[role] = false;
            filter_ready[role] = false;
            ESP_LOGW(TAG, "IMU%u disconnected; probing again", role + 1);
            continue;
        }
        ImuData scaled = imu_to_mg_and_mdps(raw);
        float accel[3] = {scaled.accel.x, scaled.accel.y, scaled.accel.z};
        float gyro[3] = {scaled.gyro.pitch, scaled.gyro.roll, scaled.gyro.yaw};
        smooth_vector(filtered_accel[role], accel, filter_ready[role]);
        smooth_vector(filtered_gyro[role], gyro, filter_ready[role]);
        filter_ready[role] = true;
        frame.connected_mask |= (uint8_t)(1U << role);
        if (role == 0) {
            frame.imu1 = scaled;
            frame.imu1_time_us = timestamp_us;
        } else {
            frame.imu2 = scaled;
            frame.imu2_time_us = timestamp_us;
        }
        if (usb) {
            printf("IMU%u,%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", role + 1, (long long)timestamp_us,
                   filtered_accel[role][0], filtered_accel[role][1], filtered_accel[role][2],
                   filtered_gyro[role][0], filtered_gyro[role][1], filtered_gyro[role][2]);
        }
    }
    if (ble && frame.connected_mask != 0) {
        ble_send_imu_frame(&frame);
    }
}

void app_main(void)
{
    bool use_emg = SENSOR_MODE == SENSOR_MODE_EMG || SENSOR_MODE == SENSOR_MODE_EMG_IMU;
    const bool use_imu = SENSOR_MODE == SENSOR_MODE_IMU || SENSOR_MODE == SENSOR_MODE_EMG_IMU;
    const bool use_usb = TRANSPORT_MODE == TRANSPORT_USB || TRANSPORT_MODE == TRANSPORT_BOTH;
    const bool use_ble = TRANSPORT_MODE == TRANSPORT_BLE || TRANSPORT_MODE == TRANSPORT_BOTH;
    if (use_imu) {
        scan_imu_bus();
    }
    if (use_ble) {
        ESP_ERROR_CHECK(app_ble_init());
    }
    if (use_usb) {
        printf("sensor,timestamp_us,data...\n");
    }
    /* Start the 100 ms ADC buffer after bus and BLE startup have finished. */
    if (use_emg) {
        ESP_ERROR_CHECK(emg_init());
    }

    uint16_t samples[EMG_BLOCK_SIZE];
    uint64_t sample_index = 0;
    int64_t next_emg_us = 0;
    uint32_t emg_sequence = 0;
    uint32_t imu_sequence = 0;
    int64_t next_imu_us = esp_timer_get_time();
    while (true) {
        if (use_emg) {
            size_t count = 0;
            esp_err_t result =
                emg_read_samples(samples, EMG_BLOCK_SIZE, &count, EMG_IMU_READ_TIMEOUT_MS);
            if (result == ESP_OK && count > 0) {
                const uint32_t period_us = 1000000U / EMG_SAMPLE_RATE_HZ;
                if (next_emg_us == 0) {
                    next_emg_us = esp_timer_get_time() - (int64_t)(count - 1) * period_us;
                }
                const int64_t first_us = next_emg_us;
                next_emg_us += (int64_t)count * period_us;
                emg_block_t block = {
                    .sequence = emg_sequence++,
                    .first_sample_time_us = (uint64_t)first_us,
                    .sample_period_us = period_us,
                    .sample_count = count,
                };
                for (size_t i = 0; i < count; ++i) {
                    block.samples[i] = samples[i];
                    if (use_usb && (sample_index % EMG_USB_PRINT_DIVIDER) == 0) {
                        printf("EMG,%lld,%u\n", (long long)(first_us + i * period_us), samples[i]);
                    }
                    ++sample_index;
                }
                if (use_ble) {
                    ble_send_emg_block(&block);
                }
            } else if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
                /* ADC pool overflow means an unknown number of conversions were
                 * lost. V1 has no ADC loss counter: stop this stream instead of
                 * continuing its synthetic sample clock across that loss. */
                ESP_LOGE(TAG, "EMG capture stopped: %s. Restart the board before recording.",
                         esp_err_to_name(result));
                use_emg = false;
                emg_deinit();
            }
        }

        int64_t now_us = esp_timer_get_time();
        if (use_imu && now_us >= next_probe_us &&
            (!imu_bus_ready || !imu_configured[0] || !imu_configured[1])) {
            scan_imu_bus();
        }
        if (use_imu && now_us >= next_imu_us) {
            read_imu_frame(use_usb, use_ble, &imu_sequence);
            next_imu_us += IMU_SAMPLE_PERIOD_MS * 1000LL;
            if (next_imu_us < esp_timer_get_time()) {
                next_imu_us = esp_timer_get_time() + IMU_SAMPLE_PERIOD_MS * 1000LL;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
