#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void bitSizeMode(int i);

void busInitalize(spi_bus_config_t *busHandle);

int amt22DoublePoll(gpio_num_t csPinNumber, uint8_t secondaryCommand, uint16_t *raw);

void amt22ReadEncoder(gpio_num_t id, uint16_t *raw);
void amt22ZeroEncoder(gpio_num_t id, uint16_t *raw);
void amt22ResetEncoder(gpio_num_t id, uint16_t *raw);