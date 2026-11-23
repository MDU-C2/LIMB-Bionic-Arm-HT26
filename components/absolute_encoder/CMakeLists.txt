if(COMMAND idf_component_register)
    idf_component_register(
        SRCS "src/encoder.c"
        INCLUDE_DIRS "include"
        REQUIRES esp_driver_gpio esp_driver_spi esp_rom freertos log
        PRIV_REQUIRES limb_utils
    )
else()
    # Keep the standalone encoder test available when configuring this directory directly.
    cmake_minimum_required(VERSION 3.16.0)
    list(APPEND EXTRA_COMPONENT_DIRS
        "${CMAKE_CURRENT_LIST_DIR}/src"
        "${CMAKE_CURRENT_LIST_DIR}/../limb_utils"
    )
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(absolutEncoder)
endif()
