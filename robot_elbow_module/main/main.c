#include <inttypes.h>
#include <math.h>

#include "can_driver.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "elbow_feedback.h"
#include "encoder.h"

// Legacy potentiometer ADC headers. The elbow now expects an EncoderAngle.
// #include "adc_manager.h"
// #include "hal/adc_types.h"
// #include "potentiometer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"
#include "imu.h"
#include "limb_utils.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "stepper.h"

static const char* const TAG = "robot_elbow_module";

// Manual motor-test branch: run the elbow from CAN without encoder feedback.
static const bool kEnableOpenLoopControl = true;
static const float kManualTestMaxAbsAngleDeg = 10.0f;
static const float kManualTestMaxSpeedDps = 1.0f;

enum {
  // Elbow V3.0 CAN transceiver connector J5.
  CAN_TX_PIN = GPIO_NUM_1,
  CAN_RX_PIN = GPIO_NUM_0,
  CAN_BAUDRATE = 1000000,

  // Elbow V3.0 SPI connector J6. Pin 1 (MISO) is unused by this link.
  ELBOW_LINK_SCLK_GPIO = GPIO_NUM_2,  // J6 pin 4
  ELBOW_LINK_CS_GPIO = GPIO_NUM_3,    // J6 pin 3
  ELBOW_LINK_MOSI_GPIO = GPIO_NUM_4,  // J6 pin 2

  // GPIO pin definitions
  // The enable pin is enabled by default.
  STEPPER_ELBOW_ENABLE_PIN = GPIO_NUM_NC,
  STEPPER_ELBOW_DIR_PIN = GPIO_NUM_6,
  STEPPER_ELBOW_STEP_PIN = GPIO_NUM_7,

  // Legacy potentiometer ADC channel.
  // // ADC channels
  // ADC_ELBOW_CHANNEL = ADC_CHANNEL_2,  // GPIO 2

  // PWM timers and channels
  PWM_ELBOW_CHANNEL = LEDC_CHANNEL_0,
  PWM_ELBOW_TIMER = LEDC_TIMER_0,

  TASK_STACK_DEPTH = 4096,
  TASK_STEPPER_UPDATE_PRIORITY = 6,
  TASK_CAN_RX_PRIORITY = 5,
  TASK_IMU_PRIORITY = 4,
};

#if CONFIG_IMU_ENABLED
// Elbow V3.0 has no assigned I2C pins for an IMU.
static const gpio_num_t IMU_SDA_PIN = GPIO_NUM_NC;
static const gpio_num_t IMU_SCL_PIN = GPIO_NUM_NC;
#endif

// Stepper configurations
static const stepper_control_config_t s_elbow_stepper_cfg = {
    .step_gpio = STEPPER_ELBOW_STEP_PIN,
    .dir_gpio = STEPPER_ELBOW_DIR_PIN,
    .enable_gpio = STEPPER_ELBOW_ENABLE_PIN,

    // Set these if you want microstepping.
    .microstepping_type = MICROSTEP_HARDWARE,
    .microstepping_mode = MICROSTEP_1_32,
    // We're setting the stepper driver's pins in hardware, so we don't need to
    // use them in the software.
    .microstep_m0_gpio = GPIO_NUM_NC,
    .microstep_m1_gpio = GPIO_NUM_NC,
    .microstep_m2_gpio = GPIO_NUM_NC,

    .direction = STEPPER_DIR_REVERSE,
    .steps_per_rev = 200,
    .gear_ratio = 15.0F,
    .max_speed_decreasing_angle = {40.0F},
    .max_speed_increasing_angle = {40.0F},
    .max_accel = {20.0F},
    .pwm_channel = PWM_ELBOW_CHANNEL,
    .pwm_timer = PWM_ELBOW_TIMER,
    // Legacy calibration belongs to the potentiometer.
    // .potentiometer =
    //     (Potentiometer){
    //         .range_of_motion = {285.F},
    //         // We step down voltage we feed to the potentiometer from 3300 mV to
    //         // 2200 mV.
    //         .min_adc_value = 0,
    //         .max_adc_value = 2200,
    //         .min_potentiometer_angle_as_joint_angle = {0.F},
    //         // Angle increases as the elbow is pulled in.
    //         .min_potentiometer_angle = {123.F},
    //         // Using the ratio between the joint angles and potentiomter angles,
    //         // 72 degrees of potentiometer angle becomes 60 degrees
    //         // of joint angle.
    //         .max_potentiometer_angle = {195.F},
    //         .joint_angle_to_potentiometer_angle_ratio = 18.F / 15.F,
    //         .is_reversed = true,
    //     },

    // Synthetic encoder frame used only by the encoder-free manual test.
    // Encoder 110 degrees is joint zero; the allowed test range is +/-10 degrees.
    .encoder = {
        .range_of_motion = {285.0F},
        .min_encoder_angle_as_joint_angle = {-10.0F},
        .min_encoder_angle = {100.0F},
        .max_encoder_angle = {120.0F},
        .joint_angle_to_encoder_angle_ratio = 1.0F,
    },
};

// Legacy ADC buffers retained for reference.
// // ADC manager configuration.
// static const AdcMgrChannelConfig s_adc_mgr_channel_configs[] = {
//     {
//         .channel = ADC_ELBOW_CHANNEL,
//         .sample_rate = 1000,
//     },
// };

// static const AdcMgrConfig s_adc_mgr_config = {
//     .channel_configs = s_adc_mgr_channel_configs,
//     .channel_configs_len = LIMB_ARR_LEN(s_adc_mgr_channel_configs),
//     .ms_worth_of_buffer_size = 100,
// };

// // NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
// // Assuming a sample rate of 1 kHz, we can comfortably keep a buffer of 1
// // seconds worth of data.
// enum {
//   ADC_STEPPERS_UNDERLYING_BUF_SIZE = 1000,
// };

// static uint16_t
//     s_adc_elbow_channel_underlying_buffer[ADC_STEPPERS_UNDERLYING_BUF_SIZE] = {
//         0};

// static AdcMgrReadResults s_adc_mgr_read_results = {
//     .channel_buffers = {
//         [ADC_ELBOW_CHANNEL] = {.data = s_adc_elbow_channel_underlying_buffer,
//                                .capacity = LIMB_ARR_LEN(
//                                    s_adc_elbow_channel_underlying_buffer)},
//     }};

// static AdcMgrChannelBuffer* s_adc_mgr_elbow_buffer =
//     &s_adc_mgr_read_results.channel_buffers[ADC_ELBOW_CHANNEL];

// static uint16_t s_latest_potentiometer_adc_value;

static stepper_control_handle_t s_elbow_stepper_handle = {0};
static QueueHandle_t s_elbow_feedback_queue;
static EncoderAngle s_open_loop_estimated_encoder_angle = {110.0F};
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

static void can_rx_task([[maybe_unused]] void* pvParameter) {
  uint8_t msg_rx[CAN_MAX_MESSAGE_SIZE];
  uint8_t rx_len = CAN_MAX_MESSAGE_SIZE;
  uint32_t rx_id = 0;

  while (1) {
    esp_err_t err = can_receive(&rx_id, msg_rx, &rx_len, portMAX_DELAY);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Error calling can_receive: %s", esp_err_to_name(err));
      continue;
    }

    switch (rx_id) {
      case CAN_ID_ROBOT_ELBOW_UP_DOWN_ACTUATION: {
        enum {
          kExpectedPayloadSize = 2 * sizeof(float),
        };
        if (rx_len != kExpectedPayloadSize) {
          ESP_LOGW(
              TAG,
              "Invalid actuation payload length for elbow: %u (expected %u)",
              rx_len, kExpectedPayloadSize);
          break;
        }

        JointAngle target_angle = {
            deserialize_float(msg_rx, kFromLittleEndian)};
        AngularVelocity target_velocity = {
            deserialize_float(msg_rx + sizeof(float), kFromLittleEndian)};
        if (!isfinite(target_angle.degree) ||
            !isfinite(target_velocity.dps) || target_velocity.dps <= 0.0F ||
            (kEnableOpenLoopControl &&
             (fabsf(target_angle.degree) > kManualTestMaxAbsAngleDeg ||
              target_velocity.dps > kManualTestMaxSpeedDps))) {
          ESP_LOGW(TAG, "Rejected unsafe elbow manual-test command");
          break;
        }
        stepper_set_target_angle(s_elbow_stepper_handle, target_angle);
        stepper_set_target_velocity(s_elbow_stepper_handle, target_velocity);
        if (kEnableOpenLoopControl) {
          // A fresh START command explicitly rearms an earlier CAN STOP.
          stepper_set_estop(s_elbow_stepper_handle, false);
        }
        ESP_LOGI(TAG,
                 "Received elbow actuation: angle=%f degrees, velocity=%f dps",
                 target_angle.degree, target_velocity.dps);
        break;
      }
      case CAN_ID_ROBOT_ELBOW_UP_DOWN_STOP: {
        stepper_set_estop(s_elbow_stepper_handle, true);
        ESP_LOGI(TAG, "Received stop command: 0x%" PRIx32, rx_id);
        break;
      }
      default: {
        ESP_LOGW(TAG, "Received unknown CAN message: 0x%" PRIx32, rx_id);
      }
    }
  }
}

[[maybe_unused]]
static void imu_task([[maybe_unused]] void* pvParameter) {
  ImuRawData raw_data;

  uint32_t can_error_count = 0;
  uint32_t can_error_count_since_last_log = 0;
  esp_err_t err = ESP_OK;

  TickType_t current_tick = xTaskGetTickCount();
  while (1) {
    // Period of 100 Hz / 10 ms.
    xTaskDelayUntil(&current_tick, pdMS_TO_TICKS(10));

    // Read IMU data.
    err = imu_read_data(&raw_data);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "Error reading IMU: %s", esp_err_to_name(err));
      continue;
    }

    const ImuData data = imu_to_mg_and_mdps(raw_data);

    // We first copy the floats we want to send to a buffer so we can reverse
    // the bytes if necessary to guarantee that we send them in little-endian
    // byte order.
    float can_buf[1] = {0};

    can_buf[0] = htolef(data.gyro.pitch);
    err = can_send(CAN_ID_ROBOT_UPPER_ARM_IMU_GYRO_PITCH, (uint8_t*)can_buf,
                   sizeof(can_buf), 0);
    if (err != ESP_OK) {
      ++can_error_count_since_last_log;
    }

    can_buf[0] = htolef(data.gyro.roll);
    err = can_send(CAN_ID_ROBOT_UPPER_ARM_IMU_GYRO_ROLL, (uint8_t*)&can_buf,
                   sizeof(can_buf), 0);
    if (err != ESP_OK) {
      ++can_error_count_since_last_log;
    }

    can_buf[0] = htolef(data.gyro.yaw);
    err = can_send(CAN_ID_ROBOT_UPPER_ARM_IMU_GYRO_YAW, (uint8_t*)&can_buf,
                   sizeof(can_buf), 0);
    if (err != ESP_OK) {
      ++can_error_count_since_last_log;
    }

    can_buf[0] = htolef(data.accel.x);
    err = can_send(CAN_ID_ROBOT_UPPER_ARM_IMU_ACCEL_X, (uint8_t*)&can_buf,
                   sizeof(can_buf), 0);
    if (err != ESP_OK) {
      ++can_error_count_since_last_log;
    }

    can_buf[0] = htolef(data.accel.y);
    err = can_send(CAN_ID_ROBOT_UPPER_ARM_IMU_ACCEL_Y, (uint8_t*)&can_buf,
                   sizeof(can_buf), 0);
    if (err != ESP_OK) {
      ++can_error_count_since_last_log;
    }

    can_buf[0] = htolef(data.accel.z);
    err = can_send(CAN_ID_ROBOT_UPPER_ARM_IMU_ACCEL_Z, (uint8_t*)&can_buf,
                   sizeof(can_buf), 0);
    if (err != ESP_OK) {
      ++can_error_count_since_last_log;
    }

    enum {
      kMinCanErrorCountPerLogging = 100,
    };
    if (can_error_count_since_last_log > kMinCanErrorCountPerLogging) {
      can_error_count += can_error_count_since_last_log;
      can_error_count_since_last_log = 0;
      ESP_LOGW(TAG, "CAN errors: %" PRIu32 ", last_error: %s", can_error_count,
               esp_err_to_name(err));
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

typedef struct {
  EncoderAngle angle;
  TickType_t received_at;
} ElbowFeedbackSample;

static void elbow_feedback_rx_task([[maybe_unused]] void* arg) {
  // Leave space beyond a frame so the driver can report excess clock bits.
  uint8_t frame[2 * ELBOW_FEEDBACK_FRAME_SIZE];
  for (;;) {
    spi_slave_transaction_t transaction = {
        .length = sizeof(frame) * 8,
        .rx_buffer = frame,
    };
    esp_err_t err = spi_slave_transmit(SPI2_HOST, &transaction, portMAX_DELAY);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Elbow SPI receiver stopped: %s", esp_err_to_name(err));
      break;
    }

    uint16_t counts;
    if (transaction.trans_len == ELBOW_FEEDBACK_FRAME_SIZE * 8 &&
        elbow_feedback_decode(frame, &counts)) {
      const ElbowFeedbackSample sample = {
          .angle = {.degree = elbow_feedback_degrees(counts)},
          .received_at = xTaskGetTickCount(),
      };
      xQueueOverwrite(s_elbow_feedback_queue, &sample);
    }
  }
  vTaskDelete(NULL);
}

static bool init_elbow_feedback_spi(void) {
  if (!GPIO_IS_VALID_GPIO(ELBOW_LINK_SCLK_GPIO) ||
      !GPIO_IS_VALID_GPIO(ELBOW_LINK_MOSI_GPIO) ||
      !GPIO_IS_VALID_GPIO(ELBOW_LINK_CS_GPIO) ||
      ELBOW_LINK_SCLK_GPIO == ELBOW_LINK_MOSI_GPIO ||
      ELBOW_LINK_SCLK_GPIO == ELBOW_LINK_CS_GPIO ||
      ELBOW_LINK_MOSI_GPIO == ELBOW_LINK_CS_GPIO) {
    ESP_LOGW(TAG, "Elbow SPI feedback disabled: assign SCLK, MOSI and CS GPIOs");
    return false;
  }

  const spi_bus_config_t bus_config = {
      .sclk_io_num = ELBOW_LINK_SCLK_GPIO,
      .mosi_io_num = ELBOW_LINK_MOSI_GPIO,
      .miso_io_num = GPIO_NUM_NC,
      .quadwp_io_num = GPIO_NUM_NC,
      .quadhd_io_num = GPIO_NUM_NC,
      .data4_io_num = GPIO_NUM_NC,
      .data5_io_num = GPIO_NUM_NC,
      .data6_io_num = GPIO_NUM_NC,
      .data7_io_num = GPIO_NUM_NC,
  };
  const spi_slave_interface_config_t slave_config = {
      .spics_io_num = ELBOW_LINK_CS_GPIO,
      .mode = ELBOW_FEEDBACK_SPI_MODE,
      .queue_size = 1,
  };
  esp_err_t err =
      spi_slave_initialize(SPI2_HOST, &bus_config, &slave_config, SPI_DMA_DISABLED);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize elbow feedback SPI: %s", esp_err_to_name(err));
    return false;
  }

  s_elbow_feedback_queue = xQueueCreate(1, sizeof(ElbowFeedbackSample));
  if (s_elbow_feedback_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create elbow feedback queue");
    spi_slave_free(SPI2_HOST);
    return false;
  }
  if (xTaskCreate(elbow_feedback_rx_task, "elbow_feedback", 3072, NULL, 7, NULL) !=
      pdPASS) {
    ESP_LOGE(TAG, "Failed to create elbow feedback receiver task");
    vQueueDelete(s_elbow_feedback_queue);
    spi_slave_free(SPI2_HOST);
    return false;
  }
  return true;
}

static bool read_elbow_encoder_angle(EncoderAngle* angle, TickType_t wait) {
  ElbowFeedbackSample sample;
  if (xQueuePeek(s_elbow_feedback_queue, &sample, wait) != pdTRUE ||
      xTaskGetTickCount() - sample.received_at >=
          pdMS_TO_TICKS(ELBOW_FEEDBACK_TIMEOUT_MS)) {
    return false;
  }
  *angle = sample.angle;
  return true;
}

static void stepper_task([[maybe_unused]] void* pvParameter) {
  TickType_t last_wake_time = xTaskGetTickCount();
  bool feedback_stop_latched = false;

  while (1) {
    const uint16_t dt_ms = 10;
    xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(dt_ms));

    // Legacy ADC update retained for reference.
    // adc_mgr_read(&s_adc_mgr_read_results, 0);
    // s_latest_potentiometer_adc_value = moving_average16(
    //     s_latest_potentiometer_adc_value, s_adc_mgr_elbow_buffer->data,
    //     s_adc_mgr_elbow_buffer->length);
    // s_adc_mgr_elbow_buffer->length = 0;
    // stepper_update(s_elbow_stepper_handle, dt_ms,
    //                s_latest_potentiometer_adc_value);

    // Legacy potentiometer-to-joint conversion.
    // // Send current angle over CAN.
    // PotentiometerAngle current_pot_angle =
    //     stepper_get_current_angle(s_elbow_stepper_handle);
    // JointAngle current_angle =
    //     to_joint_angle(&s_elbow_stepper_cfg.potentiometer, current_pot_angle);

    EncoderAngle current_encoder_angle;
    if (!read_elbow_encoder_angle(&current_encoder_angle, 0)) {
      if (!feedback_stop_latched) {
        stepper_set_estop(s_elbow_stepper_handle, true);
        feedback_stop_latched = true;
        ESP_LOGE(TAG, "Elbow motor stopped: encoder feedback unavailable");
      }
      continue;
    }
    if (!feedback_stop_latched) {
      stepper_update(s_elbow_stepper_handle, dt_ms, current_encoder_angle);
    }

    // Keep the existing CAN message ID and publish the current joint angle.
    const JointAngle current_joint_angle =
        to_joint_angle(&s_elbow_stepper_cfg.encoder, current_encoder_angle);
    float can_angle = htolef(current_joint_angle.degree);
    esp_err_t err = can_send(CAN_ID_ROBOT_ELBOW_UP_DOWN_POTENTIOMETER,
                             (uint8_t*)&can_angle, sizeof(can_angle), 0);
    static unsigned int can_errors_count = 0;
    if (err != ESP_OK) {
      if (can_errors_count++ % 100 == 0) {
        ESP_LOGW(
            TAG,
            "Error sending elbow status over CAN: %s, total error count: %u",
            esp_err_to_name(err), can_errors_count);
      }
    }

    static uint32_t status_counter = 0;
    enum {
      ITERATIONS_PER_LOGGING = 10,
    };
    if (++status_counter >= ITERATIONS_PER_LOGGING) {  // Every 100ms
      status_counter = 0;
      // Legacy target conversion used the potentiometer calibration.
      // PotentiometerAngle target_pot_angle =
      //     stepper_get_target_angle(s_elbow_stepper_handle);
      // JointAngle target_angle =
      //     to_joint_angle(&s_elbow_stepper_cfg.potentiometer, target_pot_angle);

      const EncoderAngle target_encoder_angle =
          stepper_get_target_angle(s_elbow_stepper_handle);
      const JointAngle target_joint_angle =
          to_joint_angle(&s_elbow_stepper_cfg.encoder, target_encoder_angle);
      AngularVelocity velocity =
          stepper_get_current_velocity(s_elbow_stepper_handle);
      bool moving = stepper_is_moving(s_elbow_stepper_handle);

      // Legacy log reported the ADC sample and potentiometer angle.
      // ESP_LOGI(TAG,
      //          "adc: %u, Stepper elbow - Current(pot): %.2f°, Target(pot): "
      //          "%.2f°, Current(Joint): %.2f, Target(Joint): %.2f, Velocity: "
      //          "%.2f°/s, Moving: %s",
      //          s_latest_potentiometer_adc_value, current_pot_angle.degree,
      //          target_pot_angle.degree, current_angle.degree,
      //          target_angle.degree, velocity.dps, moving ? "Yes" : "No");

      ESP_LOGI(TAG,
               "Elbow current: %.2f encoder deg, %.2f joint deg. "
               "Target: %.2f encoder deg, %.2f joint deg. Speed: %.2f deg/s. Moving: %s",
               current_encoder_angle.degree, current_joint_angle.degree,
               target_encoder_angle.degree, target_joint_angle.degree,
               velocity.dps, moving ? "Yes" : "No");
    }
  }
}

static void init_elbow_can(void) {
  const CanMsgFilter can_filter = {
      .id = CAN_RECIPIENT_ROBOT_ELBOW,
      .ignore_mask = create_filter_mask(CAN_MESSAGE_TYPE_FILTER_ANY,
                                        CAN_RECIPIENT_FILTER_EXACT,
                                        CAN_GENERIC_FILTER_ANY),
  };
  const esp_err_t err =
      can_init(CAN_TX_PIN, CAN_RX_PIN, CAN_BAUDRATE, &can_filter);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize CAN driver: %s", esp_err_to_name(err));
    abort();
  }
}

// Encoder-free test loop. The estimate starts at joint zero on every boot and
// follows the velocity sent to the existing STEP/DIR generator.
static void open_loop_stepper_task([[maybe_unused]] void* pvParameter) {
  const uint16_t dt_ms = 10;
  TickType_t last_wake_time = xTaskGetTickCount();
  uint32_t can_errors_count = 0;
  uint32_t status_counter = 0;

  for (;;) {
    xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(dt_ms));
    stepper_update(s_elbow_stepper_handle, dt_ms,
                   s_open_loop_estimated_encoder_angle);

    const AngularVelocity velocity =
        stepper_get_current_velocity(s_elbow_stepper_handle);
    s_open_loop_estimated_encoder_angle.degree +=
        velocity.dps * ((float)dt_ms / 1000.0F);
    s_open_loop_estimated_encoder_angle = clamp_encoder_angle(
        &s_elbow_stepper_cfg.encoder, s_open_loop_estimated_encoder_angle);

    const JointAngle estimated_joint_angle = to_joint_angle(
        &s_elbow_stepper_cfg.encoder, s_open_loop_estimated_encoder_angle);
    const float can_angle = htolef(estimated_joint_angle.degree);
    const esp_err_t err = can_send(CAN_ID_ROBOT_ELBOW_UP_DOWN_POTENTIOMETER,
                                   (const uint8_t*)&can_angle,
                                   sizeof(can_angle), 0);
    if (err != ESP_OK && can_errors_count++ % 100 == 0) {
      ESP_LOGW(TAG, "Error sending estimated elbow angle: %s, count: %" PRIu32,
               esp_err_to_name(err), can_errors_count);
    }

    if (++status_counter >= 100) {
      status_counter = 0;
      ESP_LOGI(TAG, "Manual test estimate: %.2f deg, speed: %.2f deg/s",
               estimated_joint_angle.degree, velocity.dps);
    }
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Robot elbow module starting...");

  if (kEnableOpenLoopControl) {
    ESP_LOGW(TAG, "ENCODER-FREE MANUAL TEST MODE: boot position is joint 0 deg");
    esp_err_t stepper_error =
        stepper_init(&s_elbow_stepper_cfg,
                     s_open_loop_estimated_encoder_angle,
                     &s_elbow_stepper_handle);
    if (stepper_error != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize elbow test stepper: %s",
               esp_err_to_name(stepper_error));
      abort();
    }
    init_elbow_can();

    if (xTaskCreate(can_rx_task, "can_rx", TASK_STACK_DEPTH, NULL,
                    TASK_CAN_RX_PRIORITY, NULL) != pdPASS ||
        xTaskCreate(open_loop_stepper_task, "open_loop_stepper",
                    TASK_STACK_DEPTH, NULL, TASK_STEPPER_UPDATE_PRIORITY,
                    NULL) != pdPASS) {
      ESP_LOGE(TAG, "Failed to create encoder-free elbow tasks");
      abort();
    }
#if CONFIG_FORCE_REENABLE_CAN_ON_BUS_OFF
    if (xTaskCreate(reenable_can_task, "reenable_can_task", TASK_STACK_DEPTH,
                    NULL, TASK_CAN_RX_PRIORITY + 1, NULL) != pdPASS) {
      ESP_LOGE(TAG, "Failed to create reenable_can_task");
      abort();
    }
#endif
    return;
  }

  const Encoder* calibration = &s_elbow_stepper_cfg.encoder;
  const float encoder_ratio = calibration->joint_angle_to_encoder_angle_ratio;
  const bool calibration_valid =
      isfinite(encoder_ratio) && encoder_ratio > 0.0F &&
      isfinite(calibration->range_of_motion.degree) &&
      calibration->range_of_motion.degree > 0.0F &&
      isfinite(calibration->min_encoder_angle_as_joint_angle.degree) &&
      isfinite(calibration->min_encoder_angle.degree) &&
      isfinite(calibration->max_encoder_angle.degree) &&
      calibration->min_encoder_angle.degree < calibration->max_encoder_angle.degree;
  if (!calibration_valid) {
    ESP_LOGW(TAG, "Elbow motor startup blocked: encoder calibration is missing or invalid");
  }
  if (!init_elbow_feedback_spi() || !calibration_valid) {
    return;
  }

  ESP_LOGI(TAG, "Waiting for initial elbow encoder feedback over SPI");
  EncoderAngle initial_encoder_angle;
  while (!read_elbow_encoder_angle(&initial_encoder_angle, portMAX_DELAY)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  esp_err_t stepper_error = stepper_init(
      &s_elbow_stepper_cfg, initial_encoder_angle, &s_elbow_stepper_handle);
  if (stepper_error != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize elbow stepper: %s",
             esp_err_to_name(stepper_error));
    abort();
  }

  init_elbow_can();

#if CONFIG_IMU_ENABLED
  const bool imu_pins_assigned =
      IMU_SDA_PIN != GPIO_NUM_NC && IMU_SCL_PIN != GPIO_NUM_NC;
  if (imu_pins_assigned) {
    ImuConfig imu_cfg = IMU_CONFIG_DEFAULT();
    imu_cfg.sda_pin = IMU_SDA_PIN;
    imu_cfg.scl_pin = IMU_SCL_PIN;
    esp_err_t err = imu_init(&imu_cfg);
    if (err) {
      ESP_LOGE(TAG, "Failed to initialize IMU driver: %s",
               esp_err_to_name(err));
      abort();
    }
    ESP_LOGI(TAG, "IMU initialized (SDA=%d, SCL=%d)", imu_cfg.sda_pin,
             imu_cfg.scl_pin);
  } else {
    ESP_LOGW(TAG, "IMU disabled: no SDA and SCL GPIOs assigned for this board");
  }
#endif

  // Legacy ADC startup and initial potentiometer reading.
  // // Initialize ADC.
  // {
  //   esp_err_t err = adc_mgr_init(s_adc_mgr_config);
  //   if (err != ESP_OK) {
  //     ESP_LOGE(TAG, "Failed to initialize ADC manager: %s",
  //              esp_err_to_name(err));
  //     abort();
  //   }
  //   ESP_LOGI(TAG, "ADC manager initialized");
  // }

  // // Wait for a bit to get initial results.
  // vTaskDelay(pdMS_TO_TICKS(10));
  // adc_mgr_read(&s_adc_mgr_read_results, 0);

  // s_latest_potentiometer_adc_value = limb_average16(
  //     s_adc_mgr_elbow_buffer->data, s_adc_mgr_elbow_buffer->length);

  // // Initialize stepper motor.
  // {
  //   esp_err_t err =
  //       stepper_init(&s_elbow_stepper_cfg, s_latest_potentiometer_adc_value,
  //                    &s_elbow_stepper_handle);
  //   if (err != ESP_OK) {
  //     ESP_LOGE(TAG, "Failed to initialize elbow stepper");
  //     abort();
  //   }
  // }
  // s_adc_mgr_elbow_buffer->length = 0;

  {
    BaseType_t err = xTaskCreate(can_rx_task, "can_rx", TASK_STACK_DEPTH, NULL,
                                 TASK_CAN_RX_PRIORITY, NULL);
    if (err != pdPASS) {
      ESP_LOGE(TAG, "Failed to create can_rx task, err code: %ld", (long)err);
      abort();
    }
  }

#if CONFIG_FORCE_REENABLE_CAN_ON_BUS_OFF
  {
    BaseType_t err =
        xTaskCreate(reenable_can_task, "reenable_can_task", TASK_STACK_DEPTH,
                    NULL, TASK_CAN_RX_PRIORITY + 1, NULL);
    if (err != pdPASS) {
      ESP_LOGE(TAG, "Failed to create reenable_can_task, err code: %ld", (long)err);
      abort();
    }
  }
#endif

#if CONFIG_IMU_ENABLED
  if (imu_pins_assigned) {
    BaseType_t err = xTaskCreate(imu_task, "imu_task", TASK_STACK_DEPTH, NULL,
                                 TASK_IMU_PRIORITY, NULL);
    if (err != pdPASS) {
      ESP_LOGE(TAG, "Failed to create imu task, err code: %ld", (long)err);
      abort();
    }
  }
#endif

  {
    BaseType_t err = xTaskCreate(stepper_task, "stepper_task", TASK_STACK_DEPTH,
                                 NULL, TASK_STEPPER_UPDATE_PRIORITY, NULL);
    if (err != pdPASS) {
      ESP_LOGE(TAG, "Failed to create stepper task, err code: %ld", (long)err);
      abort();
    }
  }

  ESP_LOGI(TAG, "Tasks created, system running");
}
