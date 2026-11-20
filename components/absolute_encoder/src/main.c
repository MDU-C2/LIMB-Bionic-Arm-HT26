#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include <stdio.h>
#include <math.h>

#define MISO_PIN GPIO_NUM_1
#define MOSI_PIN GPIO_NUM_2
#define CLK_PIN GPIO_NUM_3
#define CHIP_SELECT GPIO_NUM_0


void app_main(void) {
    bitSizeMode(1); //12-bit mode
    spi_bus_config_t busHandle = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    busInitalize(&busHandle);
    gpio_config_t csConfig = {
        .pin_bit_mask = 1ULL << CHIP_SELECT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_set_level(CHIP_SELECT, 1);
    ESP_ERROR_CHECK(gpio_config(&csConfig));

    uint16_t reading;
    
    amt22ZeroEncoder(CHIP_SELECT, &reading);
    float degreeBin;
    int degree, omega, alpha;
    int previous = reading;
    int previousVel = 0;
    while(1) {
        vTaskDelay(50);
        amt22ReadEncoder(CHIP_SELECT, &reading);
        degreeBin = reading * (360.0/4096);
        degree = round(degreeBin * 100.0);
        omega = previous - degree;
        previous = degree;
        alpha = previousVel - omega;
        previousVel = omega;
        printf("angular velocity: %d | angular acceleration: %d\n", omega, alpha);
        
    }
    
    

}

