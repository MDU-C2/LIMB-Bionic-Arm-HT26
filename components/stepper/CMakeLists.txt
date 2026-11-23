idf_component_register(
  SRCS
  "src/stepper.c"
  REQUIRES
  # The stepper receives encoder angles, so its old ADC dependencies are disabled.
  # adc_manager
  # esp_adc
  absolute_encoder
  freertos
  esp_driver_gpio
  esp_driver_ledc
  limb_utils
  motor_ramping
  INCLUDE_DIRS
  "include")
