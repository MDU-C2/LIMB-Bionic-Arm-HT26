idf_component_register(
  SRCS
  "robot_shoulder_module.c"
  "open_loop_stepper.c"
  INCLUDE_DIRS
  "."
  REQUIRES
  can_driver
  absolute_encoder
  elbow_feedback
  esp_driver_spi
  imu
  stepper
  limb_utils
  PRIV_REQUIRES esp_driver_pcnt esp_driver_rmt motor_ramping)
