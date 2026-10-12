# Jonas Gerne, Majd
idf_component_register(
    SRCS
        "main.c"
        "emg.c"
        "imu.c"
        "ble.c"
        "ble_connection.c"

    INCLUDE_DIRS
        "../include"

    REQUIRES
        esp_adc
        driver
        bt
        nvs_flash
        esp_timer
)
