idf_component_register(
    SRCS
        "main.c"
        "emg.c"

    INCLUDE_DIRS
        "../include"

    REQUIRES
        esp_adc
)