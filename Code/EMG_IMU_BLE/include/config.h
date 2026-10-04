#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "hal/adc_types.h"

// ============================================================
// SYSTEM MODES
// ============================================================

typedef enum {
    SENSOR_MODE_EMG = 0,
    SENSOR_MODE_IMU,
    SENSOR_MODE_EMG_IMU
} sensor_mode_t;

typedef enum {
    TRANSPORT_USB = 0,
    TRANSPORT_BLE
} transport_mode_t;


// ============================================================
// CURRENT TEST CONFIGURATION
// ============================================================
// Change these when selecting another test mode.

#define SENSOR_MODE      SENSOR_MODE_EMG
#define TRANSPORT_MODE   TRANSPORT_USB


// ============================================================
// EMG CONFIGURATION
// ============================================================

#define EMG_ADC_UNIT       ADC_UNIT_1
#define EMG_ADC_CHANNEL    ADC_CHANNEL_0    //Choose GP. GP0 = Pin0
#define EMG_SAMPLE_RATE_HZ 4000             // Desired EMG sample rate.
#define EMG_BUFFER_MS      100              // Amount of ADC data the internal driver should be able to buffer.
#define EMG_READ_TIMEOUT_MS 50              // Timeout when requesting data from the ADC driver.
#define EMG_BLOCK_SIZE     256              // Maximum number of EMG samples returned to main at once.   // Maximum number of EMG samples returned to main at once.
#define EMG_USB_PRINT_DIVIDER 10            // USB output decimation factor.

// ============================================================
// IMU CONFIGURATION
// ============================================================

#define IMU_I2C_PORT       I2C_NUM_0

#define IMU_SDA_PIN        GPIO_NUM_2
#define IMU_SCL_PIN        GPIO_NUM_1

#define IMU_I2C_FREQ_HZ    400000

// Same roles as the existing dual-IMU main.c
#define IMU_PRIMARY_ADDR   0x6B
#define IMU_SECONDARY_ADDR 0x6A

#define IMU_SAMPLE_PERIOD_MS 10