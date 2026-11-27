set(pcnt_dependency "")
if(IDF_TARGET STREQUAL "esp32")
  set(pcnt_dependency esp_driver_pcnt)
endif()
idf_component_register(
  SRCS "open_loop_stepper.c"
  INCLUDE_DIRS "include"
  REQUIRES stepper
  PRIV_REQUIRES esp_driver_rmt motor_ramping ${pcnt_dependency})
