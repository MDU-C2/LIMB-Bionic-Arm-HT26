#include <math.h>
#include <stdlib.h>

#include "can_driver.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "elbow_feedback.h"
#include "encoder.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"
#include "limb_utils.h"
#include "soc/gpio_num.h"
#include "stepper.h"

static const char* const TAG = "Shoulder module";

//TODO: configuration need to be updated for the new encoders and stepper motors. The gear ratios and other parameters may need to be adjusted based on the specific hardware being used.

enum {
  MOTOR_UPDATE_PERIOD_MS = 10,
  MOTOR_STATUS_LOG_PERIOD = 100,
  STEPPER_UPPER_ARM_ROTATION_ENABLE_GPIO = GPIO_NUM_NC,
  STEPPER_UPPER_ARM_ROTATION_DIR_GPIO = GPIO_NUM_2,
  STEPPER_UPPER_ARM_ROTATION_STEP_GPIO = GPIO_NUM_15,

  STEPPER_UP_DOWN_ARM_ROTATION_ENABLE_GPIO = GPIO_NUM_NC,
  STEPPER_UP_DOWN_ROTATION_DIR_GPIO = GPIO_NUM_18, //Pin 17 is NC and is connected to pin 18 with a jumper on the PCB. Change to correct GPIO for UP_DOWN when board layout is complete
  STEPPER_UP_DOWN_ROTATION_STEP_GPIO = GPIO_NUM_19, //Pin 16 is NC and is connected to pin 18 with a jumper on the PCB. Change to correct GPIO for UP_DOWN when board layout is complete
  STEPPER_LEFT_RIGHT_ARM_ROTATION_ENABLE_GPIO = GPIO_NUM_NC,
  STEPPER_LEFT_RIGHT_ROTATION_DIR_GPIO = GPIO_NUM_4, //Change to correct GPIO for LEFT_RIGHT when board layout is complete
  STEPPER_LEFT_RIGHT_ROTATION_STEP_GPIO = GPIO_NUM_0, //Change to correct GPIO for LEFT_RIGHT when board layout is complete
  
  // Microstepping pins.
  // NOTE: Since we're setting the stepper driver's pins in the hardware, we
  // don't need to use these pins in software.
  STEPPER_UPPER_ARM_ROTATION_M0_GPIO = GPIO_NUM_NC,
  STEPPER_UPPER_ARM_ROTATION_M1_GPIO = GPIO_NUM_NC,
  STEPPER_UPPER_ARM_ROTATION_M2_GPIO = GPIO_NUM_NC,

  STEPPER_LEFT_RIGHT_ROTATION_M0_GPIO = GPIO_NUM_NC,
  STEPPER_LEFT_RIGHT_ROTATION_M1_GPIO = GPIO_NUM_NC,
  STEPPER_LEFT_RIGHT_ROTATION_M2_GPIO = GPIO_NUM_NC,

  STEPPER_UP_DOWN_ROTATION_M0_GPIO = GPIO_NUM_NC,
  STEPPER_UP_DOWN_ROTATION_M1_GPIO = GPIO_NUM_NC,
  STEPPER_UP_DOWN_ROTATION_M2_GPIO = GPIO_NUM_NC,

  CAN_TX_GPIO = GPIO_NUM_1,
  CAN_RX_GPIO = GPIO_NUM_3,

  // Shared SPI bus and four chip selects for the shoulder PCB.
  ENCODER_CLK_GPIO = GPIO_NUM_14,
  ENCODER_SPIMISO_GPIO = GPIO_NUM_12,
  ENCODER_SPIMOSI_GPIO = GPIO_NUM_13,
  ENCODER_CS_UPDOWN_GPIO = GPIO_NUM_26,
  ENCODER_CS_LEFT_RIGHT_GPIO = GPIO_NUM_25,
  ENCODER_CS_UPPER_ARM_GPIO = GPIO_NUM_33,
  ENCODER_CS_ELBOW_GPIO = GPIO_NUM_32,

  // Shoulder J8 pin 4 selects the elbow ESP through elbow J6 pin 3.
  ELBOW_LINK_CS_GPIO = GPIO_NUM_27,
};



// Motors are Joyit NEMA17-06

static const stepper_control_config_t kUpDownRotationStepperConfig = {
    .enable_gpio = STEPPER_UP_DOWN_ARM_ROTATION_ENABLE_GPIO,
    .dir_gpio = STEPPER_UP_DOWN_ROTATION_DIR_GPIO,
    .step_gpio = STEPPER_UP_DOWN_ROTATION_STEP_GPIO,
    .microstepping_mode = MICROSTEP_1_32,
    .microstepping_type = MICROSTEP_HARDWARE,
    .microstep_m0_gpio = STEPPER_UP_DOWN_ROTATION_M0_GPIO,
    .microstep_m1_gpio = STEPPER_UP_DOWN_ROTATION_M1_GPIO,
    .microstep_m2_gpio = STEPPER_UP_DOWN_ROTATION_M2_GPIO,
    .direction = STEPPER_DIR_NORMAL,
    .gear_ratio = 120.F,
    .max_speed_decreasing_angle = {40.F},
    .max_speed_increasing_angle = {40.F},
    .max_accel = {20.F},
    .pwm_channel = LEDC_CHANNEL_0,
    .pwm_timer = LEDC_TIMER_0,
    .steps_per_rev = 200,
    .encoder =
        {
            .range_of_motion = {285.F},
            .min_encoder_angle = {40},
            .max_encoder_angle = {160},
            .min_encoder_angle_as_joint_angle = {-60.F},
            .joint_angle_to_encoder_angle_ratio = 1.F,
        },
};

static const stepper_control_config_t kLeftRightRotationStepperConfig = {
    .enable_gpio = STEPPER_LEFT_RIGHT_ARM_ROTATION_ENABLE_GPIO,
    .dir_gpio = STEPPER_LEFT_RIGHT_ROTATION_DIR_GPIO,
    .step_gpio = STEPPER_LEFT_RIGHT_ROTATION_STEP_GPIO,
    .microstepping_mode = MICROSTEP_1_32,
    .microstepping_type = MICROSTEP_HARDWARE,
    .microstep_m0_gpio = STEPPER_LEFT_RIGHT_ROTATION_M0_GPIO,
    .microstep_m1_gpio = STEPPER_LEFT_RIGHT_ROTATION_M1_GPIO,
    .microstep_m2_gpio = STEPPER_LEFT_RIGHT_ROTATION_M2_GPIO,
    .direction = STEPPER_DIR_NORMAL,
    .gear_ratio = 1.F,
    .max_speed_decreasing_angle = {40.F},
    .max_speed_increasing_angle = {40.F},
    .max_accel = {20.F},
    .pwm_channel = LEDC_CHANNEL_1,
    .pwm_timer = LEDC_TIMER_2,
    .steps_per_rev = 200,
    .encoder =
        {
            .range_of_motion = {285.F},
            .min_encoder_angle = {40},
            .max_encoder_angle = {160},
            .min_encoder_angle_as_joint_angle = {-60.F},
            .joint_angle_to_encoder_angle_ratio = 1.F,
        },
};

static const stepper_control_config_t kUpperArmRotationStepperConfig = {
    .enable_gpio = STEPPER_UPPER_ARM_ROTATION_ENABLE_GPIO,
    .dir_gpio = STEPPER_UPPER_ARM_ROTATION_DIR_GPIO,
    .step_gpio = STEPPER_UPPER_ARM_ROTATION_STEP_GPIO,
    .microstepping_mode = MICROSTEP_1_32,
    .microstepping_type = MICROSTEP_HARDWARE,
    .microstep_m0_gpio = STEPPER_UPPER_ARM_ROTATION_M0_GPIO,
    .microstep_m1_gpio = STEPPER_UPPER_ARM_ROTATION_M1_GPIO,
    .microstep_m2_gpio = STEPPER_UPPER_ARM_ROTATION_M2_GPIO,
    .direction = STEPPER_DIR_NORMAL,
    .gear_ratio = 15.F,
    .max_speed_decreasing_angle = {40.F},
    .max_speed_increasing_angle = {40.F},
    .max_accel = {20.F},
    .pwm_channel = LEDC_CHANNEL_2,
    .pwm_timer = LEDC_TIMER_1,
    .steps_per_rev = 200,
    .encoder =
        {
            .range_of_motion = {285.F},
            .min_encoder_angle = {40},
            .max_encoder_angle = {160},
            .min_encoder_angle_as_joint_angle = {-60.F},
            .joint_angle_to_encoder_angle_ratio = 1.F,
        },
};

typedef struct {
  EncoderAngle up_down;
  EncoderAngle left_right;
  EncoderAngle upper_arm_rotation;
  EncoderAngle elbow;
} ShoulderEncoderAngles;

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static QueueHandle_t s_encoder_angles_queue;
static spi_device_handle_t s_elbow_feedback_spi;
static stepper_control_handle_t s_left_right_stepper_handle;
static stepper_control_handle_t s_up_down_stepper_handle;
static stepper_control_handle_t s_upper_arm_rotation_stepper_handle;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

// Publish once each of the three shoulder motors has encoder feedback.
static void update_encoder_angles(const ShoulderEncoderAngles* angles) {
  if (!isfinite(angles->up_down.degree) || !isfinite(angles->left_right.degree) ||
      !isfinite(angles->upper_arm_rotation.degree)) {
    return;
  }

  xQueueOverwrite(s_encoder_angles_queue, angles);
}

// Set up the shared SPI bus and leave all four chip selects idle high.
static void init_encoders(void) {
  const gpio_num_t chip_selects[] = {
      ENCODER_CS_UPDOWN_GPIO,
      ENCODER_CS_LEFT_RIGHT_GPIO,
      ENCODER_CS_UPPER_ARM_GPIO,
      ENCODER_CS_ELBOW_GPIO,
  };
  uint64_t cs_pin_mask = 0;
  for (size_t i = 0; i < LIMB_ARR_LEN(chip_selects); ++i) {
    const gpio_num_t pin = chip_selects[i];
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
      ESP_LOGE(TAG, "Encoder chip-select GPIO %d is invalid for this target", pin);
      abort();
    }
    // Set the output latch high before enabling the active-low chip select.
    ESP_ERROR_CHECK(gpio_set_level(pin, 1));
    cs_pin_mask |= 1ULL << pin;
  }

  const gpio_config_t cs_config = {
      .pin_bit_mask = cs_pin_mask,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&cs_config));

  spi_bus_config_t bus_config = {
      .mosi_io_num = ENCODER_SPIMOSI_GPIO,
      .miso_io_num = ENCODER_SPIMISO_GPIO,
      .sclk_io_num = ENCODER_CLK_GPIO,
      .quadwp_io_num = GPIO_NUM_NC,
      .quadhd_io_num = GPIO_NUM_NC,
      .data4_io_num = GPIO_NUM_NC,
      .data5_io_num = GPIO_NUM_NC,
      .data6_io_num = GPIO_NUM_NC,
      .data7_io_num = GPIO_NUM_NC,
  };
  bitSizeMode(ELBOW_FEEDBACK_RESOLUTION_BITS == 12);
  busInitalize(&bus_config);
}

static void init_elbow_feedback_spi(int cs_gpio) {
  if (cs_gpio == GPIO_NUM_NC) {
    ESP_LOGW(TAG, "Elbow SPI feedback disabled: assign its chip-select GPIO");
    return;
  }
  if (!GPIO_IS_VALID_OUTPUT_GPIO(cs_gpio) ||
      cs_gpio == ENCODER_CLK_GPIO || cs_gpio == ENCODER_SPIMOSI_GPIO ||
      cs_gpio == ENCODER_SPIMISO_GPIO || cs_gpio == ENCODER_CS_UPDOWN_GPIO ||
      cs_gpio == ENCODER_CS_LEFT_RIGHT_GPIO ||
      cs_gpio == ENCODER_CS_UPPER_ARM_GPIO || cs_gpio == ENCODER_CS_ELBOW_GPIO) {
    ESP_LOGE(TAG, "Invalid or conflicting elbow SPI chip-select GPIO: %d", cs_gpio);
    return;
  }

  const spi_device_interface_config_t device_config = {
      .mode = ELBOW_FEEDBACK_SPI_MODE,
      .clock_speed_hz = ELBOW_FEEDBACK_SPI_CLOCK_HZ,
      .spics_io_num = cs_gpio,
      .queue_size = 1,
  };
  esp_err_t err =
      spi_bus_add_device(AMT22_SPI_HOST, &device_config, &s_elbow_feedback_spi);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add elbow to shared SPI bus: %s", esp_err_to_name(err));
  }
}

static void send_elbow_feedback(uint16_t counts) {
  if (s_elbow_feedback_spi == NULL) {
    return;
  }

  spi_transaction_t transaction = {
      .length = ELBOW_FEEDBACK_FRAME_SIZE * 8,
      .flags = SPI_TRANS_USE_TXDATA,
  };
  elbow_feedback_encode(transaction.tx_data, counts);
  esp_err_t err = spi_device_polling_transmit(s_elbow_feedback_spi, &transaction);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to send elbow feedback: %s", esp_err_to_name(err));
  }
}

// Convert a successful encoder read to degrees and keep the old angle on error.
static bool read_encoder_angle(gpio_num_t chip_select, EncoderAngle* angle,
                               uint16_t* raw_counts) {
  uint16_t counts;
  if (amt22ReadEncoder(chip_select, &counts) != ESP_OK ||
      counts >= ELBOW_FEEDBACK_COUNTS_PER_TURN) {
    return false;
  }

  angle->degree = elbow_feedback_degrees(counts);
  if (raw_counts != NULL) {
    *raw_counts = counts;
  }
  return true;
}

// Refresh each encoder independently using its last successful angle.
static void encoder_reader_task([[maybe_unused]] void* arg) {
  ShoulderEncoderAngles angles = {
      .up_down = {.degree = NAN},
      .left_right = {.degree = NAN},
      .upper_arm_rotation = {.degree = NAN},
      .elbow = {.degree = NAN},
  };
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    bool updated = read_encoder_angle(ENCODER_CS_UPDOWN_GPIO, &angles.up_down, NULL);
    updated |= read_encoder_angle(ENCODER_CS_LEFT_RIGHT_GPIO, &angles.left_right,
                                  NULL);
    updated |= read_encoder_angle(ENCODER_CS_UPPER_ARM_GPIO,
                                  &angles.upper_arm_rotation, NULL);
    uint16_t elbow_counts;
    if (read_encoder_angle(ENCODER_CS_ELBOW_GPIO, &angles.elbow, &elbow_counts)) {
      send_elbow_feedback(elbow_counts);
      updated = true;
    }
    if (updated) {
      update_encoder_angles(&angles);
    }

    xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTOR_UPDATE_PERIOD_MS));
  }
}

static void can_rx_task([[maybe_unused]] void* arg) {
  uint32_t can_id = 0;
  uint8_t can_buf[CAN_MAX_MESSAGE_SIZE] = {0};
  uint8_t can_buf_len = 0;

  while (true) {
    esp_err_t err = can_receive(&can_id, can_buf, &can_buf_len, portMAX_DELAY);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Error calling can_receive: %s", esp_err_to_name(err));
      continue;
    }
    switch (can_id) {
      case CAN_ID_ROBOT_SHOULDER_UP_DOWN_STOP: {
        stepper_set_estop(s_up_down_stepper_handle, true);
        break;
      }
      case CAN_ID_ROBOT_SHOULDER_LEFT_RIGHT_STOP: {
        stepper_set_estop(s_left_right_stepper_handle, true);
        break;
      }
      case CAN_ID_ROBOT_UPPER_ARM_ROTATION_STOP: {
        stepper_set_estop(s_upper_arm_rotation_stepper_handle, true);
        break;
      }
      case CAN_ID_ROBOT_SHOULDER_UP_DOWN_ACTUATION: {
        enum {
          kExpectedPayloadSize = 2 * sizeof(float),
        };
        if (can_buf_len != kExpectedPayloadSize) {
          ESP_LOGW(TAG,
                   "Invalid up/down actuation payload length: %u (expected %u)",
                   can_buf_len, kExpectedPayloadSize);
          break;
        }
        JointAngle joint_angle = {
            deserialize_float(can_buf, kFromLittleEndian)};
        AngularVelocity target_velocity = {
            deserialize_float(can_buf + sizeof(float), kFromLittleEndian)};
        ESP_LOGI(TAG, "Received up/down actuation: angle=%f, velocity=%f dps",
                 joint_angle.degree, target_velocity.dps);
        stepper_set_target_angle(s_up_down_stepper_handle, joint_angle);
        stepper_set_target_velocity(s_up_down_stepper_handle, target_velocity);
        break;
      }
      case CAN_ID_ROBOT_SHOULDER_LEFT_RIGHT_ACTUATION: {
        enum {
          kExpectedPayloadSize = 2 * sizeof(float),
        };
        if (can_buf_len != kExpectedPayloadSize) {
          ESP_LOGW(
              TAG,
              "Invalid left/right actuation payload length: %u (expected %u)",
              can_buf_len, kExpectedPayloadSize);
          break;
        }
        JointAngle joint_angle = {
            deserialize_float(can_buf, kFromLittleEndian)};
        AngularVelocity target_velocity = {
            deserialize_float(can_buf + sizeof(float), kFromLittleEndian)};
        ESP_LOGI(TAG,
                 "Received left/right actuation: angle=%f, velocity=%f dps",
                 joint_angle.degree, target_velocity.dps);
        stepper_set_target_angle(s_left_right_stepper_handle, joint_angle);
        stepper_set_target_velocity(s_left_right_stepper_handle,
                                    target_velocity);
        break;
      }
      case CAN_ID_ROBOT_UPPER_ARM_ROTATION_ACTUATION: {
        enum {
          kExpectedPayloadSize = 2 * sizeof(float),
        };
        if (can_buf_len != kExpectedPayloadSize) {
          ESP_LOGW(TAG,
                   "Invalid upper-arm rotation actuation payload length: %u "
                   "(expected %u)",
                   can_buf_len, kExpectedPayloadSize);
          break;
        }
        JointAngle joint_angle = {
            deserialize_float(can_buf, kFromLittleEndian)};
        AngularVelocity target_velocity = {
            deserialize_float(can_buf + sizeof(float), kFromLittleEndian)};
        ESP_LOGI(
            TAG,
            "Received upper-arm rotation actuation: angle=%f, velocity=%f dps",
            joint_angle.degree, target_velocity.dps);
        stepper_set_target_angle(s_upper_arm_rotation_stepper_handle,
                                 joint_angle);
        stepper_set_target_velocity(s_upper_arm_rotation_stepper_handle,
                                    target_velocity);
        break;
      }
      default: {
        ESP_LOGW(TAG, "Unknown CAN Message received: 0x%x", can_id);
        break;
      }
    }
  }

  vTaskDelete(NULL);
}

// Send the joint angle over the existing CAN ID and optionally log status.
static void send_joint_angle_status(stepper_control_handle_t handle,
                                    EncoderAngle encoder_angle, uint32_t can_id,
                                    const char* axis, bool log_status,
                                    uint32_t* can_errors_count) {
  const Encoder* encoder = &stepper_get_cfg(handle)->encoder;
  const JointAngle joint_angle = to_joint_angle(encoder, encoder_angle);

  if (log_status) {
    EncoderAngle target_encoder_angle = stepper_get_target_angle(handle);
    JointAngle target_joint_angle = to_joint_angle(encoder, target_encoder_angle);
    AngularVelocity velocity = stepper_get_current_velocity(handle);
    ESP_LOGI(TAG,
             "%s: encoder=%.2f, target encoder=%.2f, joint=%.2f, "
             "target joint=%.2f, velocity=%.2f dps, moving=%d",
             axis, encoder_angle.degree, target_encoder_angle.degree,
             joint_angle.degree, target_joint_angle.degree, velocity.dps,
             stepper_is_moving(handle));
  }

  esp_err_t err = can_send(can_id, (const uint8_t*)&joint_angle.degree,
                           sizeof(joint_angle.degree), 0);
  if (err != ESP_OK && (*can_errors_count)++ % 100 == 0) {
    ESP_LOGW(TAG, "Error sending %s angle over CAN: %s, total error count: %u",
             axis, esp_err_to_name(err), *can_errors_count);
  }
}

static void motors_update_task([[maybe_unused]] void* args) {
  TickType_t current_tick = xTaskGetTickCount();
  unsigned int log_tick = 0;
  uint32_t up_down_can_errors = 0;
  uint32_t left_right_can_errors = 0;
  uint32_t upper_arm_rotation_can_errors = 0;

  while (true) {
    xTaskDelayUntil(&current_tick, pdMS_TO_TICKS(MOTOR_UPDATE_PERIOD_MS));

    ShoulderEncoderAngles angles;
    if (xQueuePeek(s_encoder_angles_queue, &angles, 0) != pdTRUE) {
      continue;
    }

    stepper_update(s_up_down_stepper_handle, MOTOR_UPDATE_PERIOD_MS,
                   angles.up_down);
    stepper_update(s_left_right_stepper_handle, MOTOR_UPDATE_PERIOD_MS,
                   angles.left_right);
    stepper_update(s_upper_arm_rotation_stepper_handle, MOTOR_UPDATE_PERIOD_MS,
                   angles.upper_arm_rotation);

    log_tick = (log_tick + 1) % MOTOR_STATUS_LOG_PERIOD;
    send_joint_angle_status(
        s_up_down_stepper_handle, angles.up_down,
        CAN_ID_ROBOT_SHOULDER_UP_DOWN_POTENTIOMETER, "up/down", log_tick == 0,
        &up_down_can_errors);
    send_joint_angle_status(
        s_left_right_stepper_handle, angles.left_right,
        CAN_ID_ROBOT_SHOULDER_LEFT_RIGHT_POTENTIOMETER, "left/right",
        log_tick == 67, &left_right_can_errors);
    send_joint_angle_status(
        s_upper_arm_rotation_stepper_handle, angles.upper_arm_rotation,
        CAN_ID_ROBOT_UPPER_ARM_ROTATION_POTENTIOMETER, "upper arm rotation",
        log_tick == 34, &upper_arm_rotation_can_errors);

    if (log_tick == 50 && isfinite(angles.elbow.degree)) {
      ESP_LOGI(TAG, "Elbow encoder: %.2f degrees", angles.elbow.degree);
    }
  }
}

[[maybe_unused]]
static void reenable_can_task([[maybe_unused]] void* pvParameter) {
  while (true) {
    can_automatically_reenable_on_bus_off();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void app_main(void) {
  init_encoders();
  init_elbow_feedback_spi(ELBOW_LINK_CS_GPIO);

  // CAN initialization.
  {
    CanMsgFilter can_filter = {
        .id = CAN_RECIPIENT_ROBOT_SHOULDER,
        .ignore_mask = create_filter_mask(CAN_MESSAGE_TYPE_FILTER_ANY,
                                          CAN_RECIPIENT_FILTER_EXACT,
                                          CAN_GENERIC_FILTER_ANY),
    };

    esp_err_t err = can_init(CAN_TX_GPIO, CAN_RX_GPIO, 1000000, &can_filter);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error calling can_init: %s", esp_err_to_name(err));
      abort();
    }
  }

  s_encoder_angles_queue = xQueueCreate(1, sizeof(ShoulderEncoderAngles));
  if (s_encoder_angles_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create encoder angle queue");
    abort();
  }

  if (xTaskCreate(encoder_reader_task, "Encoder reader", 4096, NULL, 6, NULL) !=
      pdPASS) {
    ESP_LOGE(TAG, "Failed to create encoder reader task");
    abort();
  }

  ESP_LOGI(TAG, "Waiting for initial shoulder motor encoder angles");
  ShoulderEncoderAngles initial_angles;
  if (xQueuePeek(s_encoder_angles_queue, &initial_angles, portMAX_DELAY) !=
      pdTRUE) {
    ESP_LOGE(TAG, "Failed to read initial encoder angles");
    abort();
  }

  if (kUpDownRotationStepperConfig.step_gpio == GPIO_NUM_NC ||
      kLeftRightRotationStepperConfig.step_gpio == GPIO_NUM_NC) {
    ESP_LOGE(TAG, "Set up/down and left/right STEP GPIOs for the shoulder PCB");
    abort();
  }

  esp_err_t err = stepper_init(&kUpDownRotationStepperConfig,
                               initial_angles.up_down, &s_up_down_stepper_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error initializing up/down stepper: %s", esp_err_to_name(err));
    abort();
  }

  err = stepper_init(&kLeftRightRotationStepperConfig, initial_angles.left_right,
                     &s_left_right_stepper_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error initializing left/right stepper: %s",
             esp_err_to_name(err));
    abort();
  }

  err = stepper_init(&kUpperArmRotationStepperConfig,
                     initial_angles.upper_arm_rotation,
                     &s_upper_arm_rotation_stepper_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error initializing upper arm rotation stepper: %s",
             esp_err_to_name(err));
    abort();
  }

#if CONFIG_FORCE_REENABLE_CAN_ON_BUS_OFF
  {
    BaseType_t err = xTaskCreate(reenable_can_task, "reenable_can_task",
                                 1024 * 2 * 2, NULL, 6, NULL);
    if (err != pdPASS) {
      ESP_LOGE(TAG, "Failed to create reenable_can_task, err code: %d");
      abort();
    }
  }
#endif

  {
    BaseType_t err =
        xTaskCreate(can_rx_task, "CAN rx task", 1024 * 2 * 2, NULL, 5, NULL);
    if (err != pdPASS) {
      ESP_LOGE(TAG, "Failed to create reenable_can_task, err code: %d");
      abort();
    }
  }

  {
    BaseType_t err = xTaskCreate(motors_update_task, "Motors update task",
                                 1024 * 2 * 2, NULL, 6, NULL);
    if (err != pdPASS) {
      ESP_LOGE(TAG, "Failed to create reenable_can_task, err code: %d");
      abort();
    }
  }
}
