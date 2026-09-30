/* Dual LSM6DSO32 and EMG serial streamer for the ESP32-C3.
 *
 * The I2C register setup and scaling follow the verified LIMB-HT25 ESP-IDF
 * IMU component.  This target extends it to the two legal LSM6DSO32 addresses
 * on one bus and publishes both physical roles in one JSON line.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_transport.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN GPIO_NUM_2
#define I2C_SCL_PIN GPIO_NUM_1
#define I2C_FREQUENCY_HZ 100000
#define I2C_TIMEOUT_MS 20

#define SHOULDER_ADDRESS 0x6B
#define WRIST_ADDRESS 0x6A

/* Jonas's hardware-verified ADC_CHANNEL_0 is GPIO0 on ESP32-C3. */
#define EMG_ADC_CHANNEL ADC1_CHANNEL_0
#define EMG_GPIO_NUM GPIO_NUM_0

#define WHO_AM_I 0x0F
#define WHO_AM_I_VALUE 0x6C
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define OUT_TEMP_L 0x20

#define SAMPLE_PERIOD_MS 10
#define SERIAL_PERIOD_SAMPLES 2
#define RETRY_PERIOD_MS 1000

/* +-4 g and +-250 dps, matching LIMB-HT25 and the last hardware-verified
 * single-IMU streamer in this repository. */
#define ACCEL_G_PER_LSB 0.000122F
#define GYRO_DPS_PER_LSB 0.00875F

typedef struct {
  bool connected;
  float temperature_c;
  float accel_g[3];
  float gyro_dps[3];
} imu_sample_t;

typedef struct {
  esp_err_t error;
  uint8_t who_am_i;
} imu_probe_t;

typedef struct {
  gpio_num_t sda_pin;
  gpio_num_t scl_pin;
  const char *profile;
} i2c_bus_profile_t;

/* Keep 2/1 authoritative. GPIO0 is reserved for EMG. The two source-backed
 * I2C fallbacks remain available because neither uses that ADC pin. */
static const i2c_bus_profile_t I2C_BUS_PROFILES[] = {
    {I2C_SDA_PIN, I2C_SCL_PIN, "current-harness"},
    {GPIO_NUM_4, GPIO_NUM_5, "limb-ht25"},
    {GPIO_NUM_8, GPIO_NUM_5, "aurora-prototype"},
};
#define I2C_BUS_PROFILE_COUNT                                                \
  (sizeof(I2C_BUS_PROFILES) / sizeof(I2C_BUS_PROFILES[0]))

static int external_sda_pullup = 0;
static int external_scl_pullup = 0;

static esp_err_t write_register(uint8_t address, uint8_t reg, uint8_t value) {
  const uint8_t bytes[] = {reg, value};
  return i2c_master_write_to_device(I2C_PORT, address, bytes, sizeof(bytes),
                                    pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t read_registers(uint8_t address, uint8_t reg, uint8_t *data,
                                size_t length) {
  return i2c_master_write_read_device(
      I2C_PORT, address, &reg, 1, data, length,
      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t configure_imu(uint8_t address, imu_probe_t *probe) {
  uint8_t identity = 0;
  esp_err_t error = read_registers(address, WHO_AM_I, &identity, 1);
  probe->error = error;
  probe->who_am_i = identity;
  if (error != ESP_OK) {
    return error;
  }
  if (identity != WHO_AM_I_VALUE) {
    return ESP_ERR_NOT_FOUND;
  }

  /* Reset each sensor as the last hardware-verified AURORA scanner did. This
   * prevents a warm ESP32 reboot from inheriting stale register state. */
  if ((error = write_register(address, CTRL3_C, 0x01)) != ESP_OK) {
    probe->error = error;
    return error;
  }
  const int64_t reset_started_us = esp_timer_get_time();
  do {
    vTaskDelay(pdMS_TO_TICKS(2));
    error = read_registers(address, CTRL3_C, &identity, 1);
    if (error != ESP_OK) {
      probe->error = error;
      return error;
    }
  } while ((identity & 0x01) != 0 &&
           esp_timer_get_time() - reset_started_us < 100000LL);
  if ((identity & 0x01) != 0) {
    probe->error = ESP_ERR_TIMEOUT;
    return probe->error;
  }

  /* BDU + IF_INC. Both sensors sample at 104 Hz; serial output is 50 Hz. */
  if ((error = write_register(address, CTRL3_C, 0x44)) != ESP_OK ||
      (error = write_register(address, CTRL1_XL, 0x40)) != ESP_OK ||
      (error = write_register(address, CTRL2_G, 0x40)) != ESP_OK) {
    probe->error = error;
    return error;
  }
  vTaskDelay(pdMS_TO_TICKS(40));
  return ESP_OK;
}

static int16_t signed_value(uint8_t low, uint8_t high) {
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static bool read_imu(uint8_t address, imu_sample_t *sample,
                     imu_probe_t *probe) {
  uint8_t raw[14] = {0};
  probe->error = read_registers(address, OUT_TEMP_L, raw, sizeof(raw));
  if (probe->error != ESP_OK) {
    sample->connected = false;
    return false;
  }
  sample->connected = true;
  sample->temperature_c = 25.0F + signed_value(raw[0], raw[1]) / 256.0F;
  for (size_t axis = 0; axis < 3; ++axis) {
    sample->gyro_dps[axis] =
        signed_value(raw[2 + axis * 2], raw[3 + axis * 2]) * GYRO_DPS_PER_LSB;
    sample->accel_g[axis] =
        signed_value(raw[8 + axis * 2], raw[9 + axis * 2]) * ACCEL_G_PER_LSB;
  }
  return true;
}

static void print_sensor(const char *role, uint8_t address,
                         const imu_sample_t *sample, const imu_probe_t *probe,
                         int sda_level, int scl_level) {
  if (!sample->connected) {
    if (probe->error == ESP_OK) {
      printf("\"%s\":{\"connected\":false,\"model\":\"LSM6DSO32\","
             "\"address\":\"0x%02X\",\"error\":\"WHO_AM_I 0x%02X; "
             "expected 0x%02X (SDA=%d SCL=%d)\"}",
             role, address, probe->who_am_i, WHO_AM_I_VALUE, sda_level,
             scl_level);
    } else {
      const char *error_name =
          probe->error == ESP_FAIL ? "No I2C ACK" : esp_err_to_name(probe->error);
      printf("\"%s\":{\"connected\":false,\"model\":\"LSM6DSO32\","
             "\"address\":\"0x%02X\",\"error\":\"%s at 0x%02X "
             "(SDA=%d SCL=%d)\"}",
             role, address, error_name, address, sda_level, scl_level);
    }
    return;
  }
  printf("\"%s\":{\"connected\":true,\"model\":\"LSM6DSO32\","
         "\"address\":\"0x%02X\",\"temperature_c\":%.2f,"
         "\"accel_g\":{\"x\":%.5f,\"y\":%.5f,\"z\":%.5f},"
         "\"gyro_dps\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
         role, address, sample->temperature_c, sample->accel_g[0],
         sample->accel_g[1], sample->accel_g[2], sample->gyro_dps[0],
         sample->gyro_dps[1], sample->gyro_dps[2]);
}

static aurora_ble_imu_sample_t ble_sample(const imu_sample_t *sample) {
  return (aurora_ble_imu_sample_t){
      .connected = sample->connected,
      .accel_g = {sample->accel_g[0], sample->accel_g[1], sample->accel_g[2]},
      .gyro_dps = {sample->gyro_dps[0], sample->gyro_dps[1],
                   sample->gyro_dps[2]},
  };
}

static esp_err_t install_i2c_bus(gpio_num_t sda_pin, gpio_num_t scl_pin) {
  /* Adafruit's breakout has 10K external pull-ups. Brief internal pull-downs
   * reveal whether the powered cable actually reaches each candidate pin;
   * normal I2C operation then restores the ESP32 pull-ups as a safety net. */
  ESP_ERROR_CHECK(gpio_reset_pin(sda_pin));
  ESP_ERROR_CHECK(gpio_reset_pin(scl_pin));
  ESP_ERROR_CHECK(gpio_set_direction(sda_pin, GPIO_MODE_INPUT));
  ESP_ERROR_CHECK(gpio_set_direction(scl_pin, GPIO_MODE_INPUT));
  ESP_ERROR_CHECK(gpio_set_pull_mode(sda_pin, GPIO_PULLDOWN_ONLY));
  ESP_ERROR_CHECK(gpio_set_pull_mode(scl_pin, GPIO_PULLDOWN_ONLY));
  vTaskDelay(pdMS_TO_TICKS(5));
  external_sda_pullup = gpio_get_level(sda_pin);
  external_scl_pullup = gpio_get_level(scl_pin);

  const i2c_config_t bus_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda_pin,
      .scl_io_num = scl_pin,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_FREQUENCY_HZ,
      .clk_flags = 0,
  };
  esp_err_t error = i2c_param_config(I2C_PORT, &bus_config);
  if (error != ESP_OK) {
    return error;
  }
  return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static void probe_imus(bool *shoulder_ready, bool *wrist_ready,
                       imu_probe_t *shoulder_probe,
                       imu_probe_t *wrist_probe) {
  *shoulder_ready =
      configure_imu(SHOULDER_ADDRESS, shoulder_probe) == ESP_OK;
  *wrist_ready = configure_imu(WRIST_ADDRESS, wrist_probe) == ESP_OK;
}

void app_main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
  ESP_ERROR_CHECK(
      adc1_config_channel_atten(EMG_ADC_CHANNEL, ADC_ATTEN_DB_12));
  size_t active_bus_index = 0;
  const i2c_bus_profile_t *active_bus = &I2C_BUS_PROFILES[active_bus_index];
  ESP_ERROR_CHECK(install_i2c_bus(active_bus->sda_pin, active_bus->scl_pin));
  ESP_ERROR_CHECK(ble_transport_init());

  bool shoulder_ready = false;
  bool wrist_ready = false;
  imu_probe_t shoulder_probe = {.error = ESP_ERR_INVALID_STATE};
  imu_probe_t wrist_probe = {.error = ESP_ERR_INVALID_STATE};
  probe_imus(&shoulder_ready, &wrist_ready, &shoulder_probe, &wrist_probe);
  while (!shoulder_ready && !wrist_ready &&
         active_bus_index + 1 < I2C_BUS_PROFILE_COUNT) {
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
    active_bus = &I2C_BUS_PROFILES[++active_bus_index];
    ESP_ERROR_CHECK(install_i2c_bus(active_bus->sda_pin, active_bus->scl_pin));
    probe_imus(&shoulder_ready, &wrist_ready, &shoulder_probe, &wrist_probe);
  }
  int64_t last_retry_us = esp_timer_get_time();
  uint32_t sequence = 0;
  TickType_t last_wake = xTaskGetTickCount();

  while (true) {
    const int64_t now_us = esp_timer_get_time();
    if (now_us - last_retry_us >= RETRY_PERIOD_MS * 1000LL) {
      if (!shoulder_ready) {
        shoulder_ready =
            configure_imu(SHOULDER_ADDRESS, &shoulder_probe) == ESP_OK;
      }
      if (!wrist_ready) {
        wrist_ready = configure_imu(WRIST_ADDRESS, &wrist_probe) == ESP_OK;
      }
      /* Cycle the known harnesses when nothing answers. This also makes
       * hot-plugging work without rebooting the ESP32. */
      if (!shoulder_ready && !wrist_ready) {
        ESP_ERROR_CHECK(i2c_driver_delete(I2C_PORT));
        active_bus_index = (active_bus_index + 1) % I2C_BUS_PROFILE_COUNT;
        active_bus = &I2C_BUS_PROFILES[active_bus_index];
        ESP_ERROR_CHECK(
            install_i2c_bus(active_bus->sda_pin, active_bus->scl_pin));
        probe_imus(&shoulder_ready, &wrist_ready, &shoulder_probe,
                   &wrist_probe);
      }
      last_retry_us = now_us;
    }

    const int64_t sample_time_us = esp_timer_get_time();
    const int emg_reading = adc1_get_raw(EMG_ADC_CHANNEL);
    const bool emg_connected = emg_reading >= 0;
    const uint16_t emg_raw =
        emg_connected ? (uint16_t)emg_reading : 0U;
    imu_sample_t shoulder = {.connected = shoulder_ready};
    imu_sample_t wrist = {.connected = wrist_ready};
    if (shoulder_ready) {
      shoulder_ready =
          read_imu(SHOULDER_ADDRESS, &shoulder, &shoulder_probe);
    }
    if (wrist_ready) {
      wrist_ready = read_imu(WRIST_ADDRESS, &wrist, &wrist_probe);
    }

    const aurora_ble_imu_sample_t shoulder_ble = ble_sample(&shoulder);
    const aurora_ble_imu_sample_t wrist_ble = ble_sample(&wrist);
    ble_transport_publish(sequence, (uint64_t)sample_time_us, &shoulder_ble,
                          &wrist_ble, emg_connected, emg_raw);

    if (sequence % SERIAL_PERIOD_SAMPLES == 0) {
      const int sda_level = gpio_get_level(active_bus->sda_pin);
      const int scl_level = gpio_get_level(active_bus->scl_pin);
      printf("{\"schema\":\"aurora.sensors.v1\",\"device\":\"ESP32-C3\","
             "\"sequence\":%lu,\"sample_time_us\":%lld,"
             "\"uptime_ms\":%lld,\"free_heap_bytes\":%lu,"
             "\"i2c\":{\"sda_pin\":%d,\"scl_pin\":%d,"
             "\"profile\":\"%s\",\"fallback\":%s,"
             "\"sda_level\":%d,\"scl_level\":%d,"
             "\"external_sda_pullup\":%s,\"external_scl_pullup\":%s},"
             "\"emg\":{\"connected\":%s,\"gpio\":%d,"
             "\"adc_channel\":%d,\"adc_raw\":%u},\"imus\":{",
             (unsigned long)sequence, sample_time_us, sample_time_us / 1000LL,
             (unsigned long)esp_get_free_heap_size(), active_bus->sda_pin,
             active_bus->scl_pin, active_bus->profile,
             active_bus_index == 0 ? "false" : "true", sda_level, scl_level,
             external_sda_pullup ? "true" : "false",
             external_scl_pullup ? "true" : "false",
             emg_connected ? "true" : "false", EMG_GPIO_NUM,
             EMG_ADC_CHANNEL, emg_raw);
      print_sensor("shoulder", SHOULDER_ADDRESS, &shoulder, &shoulder_probe,
                   sda_level, scl_level);
      printf(",");
      print_sensor("wrist", WRIST_ADDRESS, &wrist, &wrist_probe, sda_level,
                   scl_level);
      printf("}}\n");
    }
    sequence++;

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}
