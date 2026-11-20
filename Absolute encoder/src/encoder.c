#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "hal/spi_types.h"
#include <cstdint>
#include <stdint.h>


#define ESP_ERR_ENCODER_PARITY_BIT_FAILURE
bool twelveBitMode;

static const char* const TAG = "encoderLowLevel";


static spi_device_handle_t amt;




spi_device_interface_config_t encoderConfig = {
    .mode = 0,
    .clock_speed_hz = 500000,
    .spics_io_num = -1,
    .queue_size = 1,
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0
};

void bitSizeMode(int i) {
    if (i == 1) twelveBitMode = true;
    else twelveBitMode = false;
}


void busInitalize(spi_bus_config_t *busHandle) {
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, busHandle, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &encoderConfig, &amt));
}

static esp_err_t amt22ChecksumVerify(uint16_t raw)
{
    // Odd parity checkbits: bit15 covers odd bits, bit14 covers even bits
    bool k1 = !(((raw >> 13) ^ (raw >> 11) ^ (raw >> 9) ^ (raw >> 7) ^
                 (raw >> 5)  ^ (raw >> 3)  ^ (raw >> 1)) & 1);
    bool k0 = !(((raw >> 12) ^ (raw >> 10) ^ (raw >> 8) ^ (raw >> 6) ^
                 (raw >> 4)  ^ (raw >> 2)  ^ (raw >> 0)) & 1);
    bool result = (k1 == ((raw >> 15) & 1)) && (k0 == ((raw >> 14) & 1));

    if (result) return ESP_OK;
    else return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t amt22ReadCommandChar(uint8_t tx, uint8_t*rxData)
{
    spi_transaction_t t = {
        .length = 8, .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .tx_data = { tx },
    };
    esp_err_t err = spi_device_polling_transmit(amt, &t);
    *rxData = t.rx_data[0];
    if (err != ESP_OK) return err; 
    else return ESP_OK;
}

esp_err_t amt22DoublePoll(gpio_num_t csPinNumber, uint8_t secondaryCommand, uint16_t *raw){
    uint8_t hByte, lByte;
    spi_device_acquire_bus(amt, portMAX_DELAY);
    gpio_set_level(csPinNumber, 0);
    esp_rom_delay_us(3);
    esp_err_t errH = amt22ReadCommandChar(0x00, &hByte);
    esp_rom_delay_us(3);
    esp_err_t errL = amt22ReadCommandChar(secondaryCommand, &lByte);
    esp_rom_delay_us(3);
    gpio_set_level(csPinNumber, 1);
    esp_rom_delay_us(40);
    spi_device_release_bus(amt);

    *raw = ((uint16_t)hByte << 8) | lByte;
    esp_err_t err = amt22ChecksumVerify(*raw);

    if(twelveBitMode == true) *raw = (*raw & 0x3FFF) >> 2; //shift 12 bits
    else *raw = *raw & 0x3FFF; 

    if (errH != ESP_OK || errL != ESP_OK || err != ESP_OK) {

        ESP_LOGW(TAG, "parity bit failure on encoder %d", csPinNumber);
        return err;
    }
    else return ESP_OK;
    
}

esp_err_t amt22ReadEncoder(gpio_num_t id, uint16_t *raw){
    esp_err_t err = amt22DoublePoll(id, 0x00, raw);
}

esp_err_t amt22ZeroEncoder(gpio_num_t id, uint16_t *raw){
    amt22DoublePoll(id, 0x70, raw);
}
esp_err_t amt22ResetEncoder(gpio_num_t id, uint16_t *raw){
    amt22DoublePoll(id, 0x60,  raw);
}