idf_component_register(SRCS "main.c" "ble_transport.c"
                       REQUIRES driver esp_timer esp_system bt nvs_flash)
