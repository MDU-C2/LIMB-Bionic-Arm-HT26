idf_component_register(
    SRCS
        "main.c"
        "emg.c"
        "imu.c"

    INCLUDE_DIRS
        "../include"

    REQUIRES
        esp_adc
        driver
)