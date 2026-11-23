idf_component_register(
    SRCS "main.c" "encoder.c" "kinematics.c"
    INCLUDE_DIRS "../include"
    REQUIRES esp_driver_gpio esp_driver_spi esp_rom freertos log limb_utils
)
