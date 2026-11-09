/* Jonas Gerne, Majd */
#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "hal/adc_types.h"

// Sensor and transport selection.
typedef enum { SENSOR_MODE_EMG = 0, SENSOR_MODE_IMU, SENSOR_MODE_EMG_IMU } sensor_mode_t;

typedef enum { TRANSPORT_USB = 0, TRANSPORT_BLE, TRANSPORT_BOTH } transport_mode_t;

// Change these when selecting another recording mode.

#define SENSOR_MODE SENSOR_MODE_EMG_IMU
#define TRANSPORT_MODE TRANSPORT_BLE

// EMG acquisition.
#define EMG_ADC_UNIT ADC_UNIT_1
#define EMG_ADC_CHANNEL ADC_CHANNEL_0 // GPIO0 on the ESP32-C3.
#define EMG_SAMPLE_RATE_HZ 4000       // Desired EMG sample rate.
#define EMG_BUFFER_MS 100             // ADC buffer duration.
#define EMG_BLOCK_SIZE 256            // Maximum number of EMG samples returned at once.
#define EMG_USB_PRINT_DIVIDER 10      // Diagnostic output only; BLE sends every sample.

// Both IMUs share the I2C bus and use different addresses.
#define IMU_I2C_PORT I2C_NUM_0
#define IMU_SDA_PIN GPIO_NUM_2
#define IMU_SCL_PIN GPIO_NUM_1
#define IMU_I2C_FREQ_HZ 400000
#define IMU_PRIMARY_ADDR 0x6B
#define IMU_SECONDARY_ADDR 0x6A
#define IMU_SAMPLE_PERIOD_MS 10

#define EMG_IMU_READ_TIMEOUT_MS 2
