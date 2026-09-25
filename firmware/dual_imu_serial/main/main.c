/* Dual LSM6DSO32 serial streamer for the ESP32-C3.
 *
 * The I2C register setup and scaling follow the verified LIMB-HT25 ESP-IDF
 * IMU component.  This target extends it to the two legal LSM6DSO32 addresses
 * on one bus and publishes both physical roles in one JSON line.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN GPIO_NUM_8
#define I2C_SCL_PIN GPIO_NUM_5
#define I2C_FREQUENCY_HZ 400000
#define I2C_TIMEOUT_MS 100

#define SHOULDER_ADDRESS 0x6A
#define WRIST_ADDRESS 0x6B

#define WHO_AM_I 0x0F
#define WHO_AM_I_VALUE 0x6C
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define OUT_TEMP_L 0x20

#define SAMPLE_PERIOD_MS 20
#define RETRY_PERIOD_MS 1000

/* +-4 g and +-500 dps, matching the final LIMB-HT25 human-arm change. */
#define ACCEL_G_PER_LSB 0.000122F
#define GYRO_DPS_PER_LSB 0.01750F

typedef struct {
  bool connected;
  float temperature_c;
  float accel_g[3];
  float gyro_dps[3];
} imu_sample_t;

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

static esp_err_t configure_imu(uint8_t address) {
  uint8_t identity = 0;
  esp_err_t error = read_registers(address, WHO_AM_I, &identity, 1);
  if (error != ESP_OK || identity != WHO_AM_I_VALUE) {
    return ESP_ERR_NOT_FOUND;
  }

  /* BDU + IF_INC.  Both sensors sample at 104 Hz; serial output is 50 Hz. */
  if ((error = write_register(address, CTRL3_C, 0x44)) != ESP_OK ||
      (error = write_register(address, CTRL1_XL, 0x40)) != ESP_OK ||
      (error = write_register(address, CTRL2_G, 0x44)) != ESP_OK) {
    return error;
  }
  vTaskDelay(pdMS_TO_TICKS(40));
  return ESP_OK;
}

static int16_t signed_value(uint8_t low, uint8_t high) {
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static bool read_imu(uint8_t address, imu_sample_t *sample) {
  uint8_t raw[14] = {0};
  if (read_registers(address, OUT_TEMP_L, raw, sizeof(raw)) != ESP_OK) {
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
                         const imu_sample_t *sample) {
  if (!sample->connected) {
    printf("\"%s\":{\"connected\":false,\"model\":\"LSM6DSO32\","
           "\"address\":\"0x%02X\",\"error\":\"I2C read failed\"}",
           role, address);
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

void app_main(void) {
  setvbuf(stdout, NULL, _IONBF, 0);
  const i2c_config_t bus_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_FREQUENCY_HZ,
      .clk_flags = 0,
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &bus_config));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

  bool shoulder_ready = configure_imu(SHOULDER_ADDRESS) == ESP_OK;
  bool wrist_ready = configure_imu(WRIST_ADDRESS) == ESP_OK;
  int64_t last_retry_us = esp_timer_get_time();

  while (true) {
    const int64_t now_us = esp_timer_get_time();
    if (now_us - last_retry_us >= RETRY_PERIOD_MS * 1000LL) {
      if (!shoulder_ready) {
        shoulder_ready = configure_imu(SHOULDER_ADDRESS) == ESP_OK;
      }
      if (!wrist_ready) {
        wrist_ready = configure_imu(WRIST_ADDRESS) == ESP_OK;
      }
      last_retry_us = now_us;
    }

    imu_sample_t shoulder = {.connected = shoulder_ready};
    imu_sample_t wrist = {.connected = wrist_ready};
    if (shoulder_ready) {
      shoulder_ready = read_imu(SHOULDER_ADDRESS, &shoulder);
    }
    if (wrist_ready) {
      wrist_ready = read_imu(WRIST_ADDRESS, &wrist);
    }

    printf("{\"schema\":\"aurora.dual_imu.v1\",\"device\":\"ESP32-C3\","
           "\"uptime_ms\":%lld,\"free_heap_bytes\":%lu,"
           "\"i2c\":{\"sda_pin\":%d,\"scl_pin\":%d},\"imus\":{",
           now_us / 1000LL, (unsigned long)esp_get_free_heap_size(), I2C_SDA_PIN,
           I2C_SCL_PIN);
    print_sensor("shoulder", SHOULDER_ADDRESS, &shoulder);
    printf(",");
    print_sensor("wrist", WRIST_ADDRESS, &wrist);
    printf("}}\n");

    vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}
