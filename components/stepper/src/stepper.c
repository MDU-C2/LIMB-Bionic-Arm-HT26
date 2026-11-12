#include "stepper.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_clk_tree.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"
#include "limb_utils.h"
#include "motor_ramping.h"
#include "portmacro.h"
#include "encoder.h"
#include "soc/gpio_num.h"
#include "soc/soc_caps.h"
#include "sys/param.h"

static const char* TAG = "stepper";

// NOTE: Without microstepping, our step size is 1.8 degrees, so the deadband
// should probably be at least as large.
/*
 * TODO: Accept absolute encoder angles directly (no potentiometer/ADC).
 * - Define a sensor-independent angle type in a shared header, with degrees,
 *   zero reference, positive direction, and wraparound behavior documented.
 *   Replace PotentiometerAngle and move/replace JointAngle so neither requires
 *   potentiometer.h; use the same coordinate system for feedback and targets. DONE

 * - Change stepper_init() and stepper_update() in stepper.c and stepper.h to
 *   receive an angle instead of latest_potentiometer_adc_value. Store that
 *   angle directly and remove both potentiometer_adc_to_angle() calls. DONE

 * - Replace the potentiometer-specific 10-degree safety check with encoder
 *   feedback validation (finite angle, valid/fresh reading, joint limits),
 *   stopping the motor on invalid feedback. Define how callers report failures.

 * - Update MotorRampingArgs and motor_ramping to use the shared angle type for
 *   current_angle, target_angle, and deadband. DONE

 * - Update angle getters, declarations, comments, and logs to use the new type
 *   and encoder terminology; remove potentiometer.h and unused ADC includes.

 * - Update callers/configurations to pass encoder angles at initialization and
 *   each update; apply any encoder zero/direction/gear mapping before passing
 *   feedback to the driver. Remove unused potentiometer/ADC build dependencies
 *   from stepper and motor_ramping after checking their remaining requirements. DONE

 * - Build affected components and verify initial/target clamping, direction,
 *   deadband stopping, ramping, invalid feedback, and emergency stop. DONE
 * 
 * - How will we use deadband if it is still needed? The deadband is severly lower with new encoders
 *  and may not be necessary and instead stopping or slowing down when arm closes on its limits may be more appropriate. We will need to test this with the new encoders and see if we still need a deadband or if we can remove it entirely.
 * 
 * - Update documentation to reflect the new angle type and encoder-based feedback.
 */
// If we're within +- 2 degrees, we stop.
#define DEADBAND_DEG 4.F

// Control context
typedef struct {
  stepper_control_config_t cfg;
  portMUX_TYPE spinlock;

  // LEDC config
  uint32_t min_frequency;
  uint32_t duty_50_percent;

  // Motion state
  bool estop_active;
  bool is_moving;
  AngularVelocity target_velocity;
  AngularVelocity current_velocity;
  EncoderAngle target_angle;
  EncoderAngle current_angle;
  bool use_position_feedback;

  // Calculated parameters
  float steps_per_degree;

  bool is_initialized;
} motion_control_context_t;

// We only support at most as many steppers as there are LEDC channels, since
// they require exclusive access anyway.
static motion_control_context_t s_contexts[SOC_LEDC_CHANNEL_NUM] = {0};

//ok
static void stop_motor(stepper_control_handle_t handle) {
  motion_control_context_t* ctx = &s_contexts[handle];

  ledc_set_duty(LEDC_LOW_SPEED_MODE, ctx->cfg.pwm_channel, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ctx->cfg.pwm_channel);

  ESP_LOGD(TAG, "Stopping motor");

  portENTER_CRITICAL(&ctx->spinlock);
  ctx->is_moving = false;
  ctx->current_velocity = (AngularVelocity){0.0F};
  portEXIT_CRITICAL(&ctx->spinlock);
}
//ok
static void apply_motor_velocity(stepper_control_handle_t handle,
                                 AngularVelocity velocity) {
  motion_control_context_t* ctx = &s_contexts[handle];

  // Enable motor
  if (ctx->cfg.enable_gpio != GPIO_NUM_NC) {
    gpio_set_level(ctx->cfg.enable_gpio, 0);  // Enable (active low)
  }

  // Direction control
  if (ctx->cfg.dir_gpio != GPIO_NUM_NC) {
    uint8_t direction = velocity.dps < 0.0F ? 1 : 0;

    // Swap the direction if we're in reverse mode.
    if (ctx->cfg.direction == STEPPER_DIR_REVERSE) {
      direction = !direction;
    }

    gpio_set_level(ctx->cfg.dir_gpio, direction);
  }

  float velocity_sps = roundf(velocity.dps * ctx->steps_per_degree);

  // Clamp frequency
  uint32_t freq_hz = MAX((uint32_t)fabsf(velocity_sps), ctx->min_frequency);

  ESP_LOGD(TAG, "Setting freq: %u Hz for %f dps", freq_hz, velocity.dps);

  // Update frequency and duty
  ledc_set_freq(LEDC_LOW_SPEED_MODE, ctx->cfg.pwm_timer, freq_hz);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ctx->cfg.pwm_channel,
                ctx->duty_50_percent);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ctx->cfg.pwm_channel);
}

// Initialization

esp_err_t stepper_init(const stepper_control_config_t* cfg,
                       EncoderAngle latest_encoder_angle,
                       stepper_control_handle_t* out_handle) {
  // Validate config
  if (!cfg) return ESP_ERR_INVALID_ARG;

  motion_control_context_t ctx = {0};

  stepper_control_handle_t handle = cfg->pwm_channel;

  // Reset and store config
  ctx.cfg = *cfg;
  ctx.spinlock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

  // We define the degrees per second and need to convert that into steps per
  // second
  const float microstepping_factor = MAX((int)ctx.cfg.microstepping_mode, 1);
  ctx.steps_per_degree = (float)cfg->steps_per_rev * cfg->gear_ratio *
                         microstepping_factor / 360.0f;

  uint32_t clk_freq = 0;
  ESP_RETURN_ON_ERROR(
      esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APB, 0, &clk_freq), TAG,
      "Couldn't get clock frequency");
  const float max_vel = MAX(ctx.cfg.max_speed_decreasing_angle.dps,
                            ctx.cfg.max_speed_increasing_angle.dps);
  const uint32_t max_freq = (uint32_t)roundf(max_vel * ctx.steps_per_degree);

  ESP_LOGI(TAG, "Using microstepping factor %u", microstepping_factor);

  // The values of the ledc_timer_bit_t enumerations correspond to their
  // bitwidths.
  const ledc_timer_bit_t duty_res =
      ledc_find_suitable_duty_resolution(clk_freq, max_freq);

  ESP_LOGI(TAG, "Selecting duty resolution %d for max frequency %u", duty_res,
           (uint32_t)max_freq);

  // Configure GPIOS for STEP, DIR and ENABLE
  uint64_t pin_mask = (1ULL << cfg->step_gpio);
  if (cfg->dir_gpio != GPIO_NUM_NC) {
    pin_mask |= (1ULL << cfg->dir_gpio);
  }
  if (cfg->enable_gpio != GPIO_NUM_NC) {
    pin_mask |= (1ULL << cfg->enable_gpio);
  }
  if (cfg->microstepping_type == MICROSTEP_SOFTWARE &&
      cfg->microstepping_mode != MICROSTEP_NONE) {
    if (cfg->microstep_m0_gpio == GPIO_NUM_NC) {
      ESP_LOGE(TAG,
               "Microstepping is enabled, but microstep_m0_gpio isn't enabled");
      return ESP_ERR_INVALID_ARG;
    } else {
      pin_mask |= (1ULL << cfg->microstep_m0_gpio);
    }
    if (cfg->microstep_m1_gpio == GPIO_NUM_NC) {
      ESP_LOGE(TAG,
               "Microstepping is enabled, but microstep_m1_gpio isn't enabled");
      return ESP_ERR_INVALID_ARG;
    } else {
      pin_mask |= (1ULL << cfg->microstep_m1_gpio);
    }
    if (cfg->microstep_m2_gpio == GPIO_NUM_NC) {
      ESP_LOGE(TAG,
               "Microstepping is enabled, but microstep_m2_gpio isn't enabled");
      return ESP_ERR_INVALID_ARG;
    } else {
      pin_mask |= (1ULL << cfg->microstep_m2_gpio);
    }
  } else if (cfg->microstepping_type == MICROSTEP_HARDWARE &&
             cfg->microstepping_mode != MICROSTEP_NONE) {
    if (cfg->microstep_m0_gpio != GPIO_NUM_NC ||
        cfg->microstep_m1_gpio != GPIO_NUM_NC ||
        cfg->microstep_m2_gpio != GPIO_NUM_NC) {
      ESP_LOGE(TAG,
               "Hardware microstepping is enabled, but a microstep gpio pin is "
               "not set to NC");
      return ESP_ERR_INVALID_ARG;
    }
  }

  gpio_config_t io_conf = {
      .pin_bit_mask = pin_mask,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure GPIOs");

  // Set initial GPIO states
  gpio_set_level(cfg->step_gpio, 0);
  if (cfg->dir_gpio != GPIO_NUM_NC) {
    gpio_set_level(cfg->dir_gpio, 0);
  }
  if (cfg->enable_gpio != GPIO_NUM_NC) {
    gpio_set_level(cfg->enable_gpio, 0);
  }  // active low on DRV8825

  // Set up microstepping.
  if (cfg->microstepping_type == MICROSTEP_SOFTWARE) {
    switch (cfg->microstepping_mode) {
      case MICROSTEP_NONE: {
        // We don't need to do anything.
        break;
      }
      case MICROSTEP_1_1: {
        gpio_set_level(cfg->microstep_m0_gpio, 0);
        gpio_set_level(cfg->microstep_m1_gpio, 0);
        gpio_set_level(cfg->microstep_m2_gpio, 0);
        break;
      }
      case MICROSTEP_1_2: {
        gpio_set_level(cfg->microstep_m0_gpio, 1);
        gpio_set_level(cfg->microstep_m1_gpio, 0);
        gpio_set_level(cfg->microstep_m2_gpio, 0);
        break;
      }
      case MICROSTEP_1_4: {
        gpio_set_level(cfg->microstep_m0_gpio, 0);
        gpio_set_level(cfg->microstep_m1_gpio, 1);
        gpio_set_level(cfg->microstep_m2_gpio, 0);
        break;
      }
      case MICROSTEP_1_8: {
        gpio_set_level(cfg->microstep_m0_gpio, 1);
        gpio_set_level(cfg->microstep_m1_gpio, 1);
        gpio_set_level(cfg->microstep_m2_gpio, 0);
        break;
      }
      case MICROSTEP_1_16: {
        gpio_set_level(cfg->microstep_m0_gpio, 0);
        gpio_set_level(cfg->microstep_m1_gpio, 0);
        gpio_set_level(cfg->microstep_m2_gpio, 1);
        break;
      }
      case MICROSTEP_1_32: {
        gpio_set_level(cfg->microstep_m0_gpio, 1);
        gpio_set_level(cfg->microstep_m1_gpio, 0);
        gpio_set_level(cfg->microstep_m2_gpio, 1);
        break;
      }
      default: {
        ESP_LOGE(TAG, "You set the microstepping_mode to invalid value (%d)",
                 cfg->microstepping_mode);
        return ESP_ERR_INVALID_ARG;
      }
    }
  }

  // Configure LEDC timer
  // Start off at lowest possible frequency (5 Hz for duty resolution 14 using
  // APB_CLK on ESP32-C3-Zero).
  const uint32_t initial_freq_hz = 5;

  ledc_timer_config_t timer_cfg = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = duty_res,
      .timer_num = cfg->pwm_timer,
      .freq_hz = initial_freq_hz,
      .clk_cfg = LEDC_USE_APB_CLK,
  };

  ESP_LOGI(TAG, "Finding min supported frequency...");
  // Different duty resolutions have different minimum frequencies that they
  // support. We increase the frequency until the library allows it.
  esp_err_t err = ledc_timer_config(&timer_cfg);
  while (err != ESP_OK) {
    // ESP_FAIL represents an invalid duty resolution and frequency combo.
    // Any other error means something is wrong.
    if (err != ESP_FAIL) {
      ESP_LOGE(TAG, "Error configuring ledc timer: %s", esp_err_to_name(err));
      return err;
    }
    ++timer_cfg.freq_hz;
    err = ledc_timer_config(&timer_cfg);
  }
  ctx.min_frequency = timer_cfg.freq_hz;
  ESP_LOGI(TAG, "Selecting min frequency %u Hz", ctx.min_frequency);

  const float min_allowed_velocity =
      (float)ctx.min_frequency / ctx.steps_per_degree;
  if (cfg->max_speed_decreasing_angle.dps < min_allowed_velocity) {
    ESP_LOGE(TAG, "max_speed_decreasing_angle must be at least %f dps",
             min_allowed_velocity);
    return ESP_ERR_INVALID_ARG;
  }
  if (cfg->max_speed_increasing_angle.dps < min_allowed_velocity) {
    ESP_LOGE(TAG, "max_speed_increasing_angle must be at least %f dps",
             min_allowed_velocity);
    return ESP_ERR_INVALID_ARG;
  }

  // Configure LEDC channel for STEP output
  ctx.duty_50_percent = (1 << (timer_cfg.duty_resolution -
                               1));  // 50% duty for set duty resolution

  ledc_channel_config_t channel_cfg = {
      .gpio_num = cfg->step_gpio,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = cfg->pwm_channel,
      .timer_sel = cfg->pwm_timer,
      .duty = 0,  // IDLE (no pulses)
      .hpoint = 0,
  };
  ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_cfg), TAG,
                      "Failed to configure LEDC channel");
  // change to just angle with no ADC conversion step
  EncoderAngle initial_angle = latest_encoder_angle;
  ctx.current_angle = initial_angle;
  // Our target angle should be within the range of allowed angles, even if
  // the current angle isn't.
  ctx.target_angle =
      clamp_encoder_angle(&ctx.cfg.encoder, ctx.current_angle);
  ctx.use_position_feedback = true;

  ESP_LOGI(TAG, "encoder initialized: angle=%.2f deg",
           latest_encoder_angle.degree, initial_angle.degree);

  // Initialize motion state
  ctx.estop_active = false;
  ctx.is_moving = false;
  ctx.current_velocity = (AngularVelocity){0.0F};
  ctx.target_velocity = (AngularVelocity){0.0F};

  ctx.is_initialized = true;

  s_contexts[handle] = ctx;

  *out_handle = handle;

  ESP_LOGI(TAG,
           "Stepper initialized: steps/deg=%.3f, max_vel_pos=%.2f sps, "
           "max_vel_neg=%.2f sps"
           "max_accel=%.2f sps²",
           ctx.steps_per_degree,
           ctx.cfg.max_speed_increasing_angle.dps * ctx.steps_per_degree,
           ctx.cfg.max_speed_decreasing_angle.dps * ctx.steps_per_degree,
           ctx.cfg.max_accel.dps2 * ctx.steps_per_degree);

  return ESP_OK;
}

//ok
esp_err_t stepper_deinit(stepper_control_handle_t handle) {
  motion_control_context_t* ctx = &s_contexts[handle];

  // Stop the motor
  stop_motor(handle);

  *ctx = (motion_control_context_t){0};

  return ESP_OK;
}

void stepper_update(stepper_control_handle_t handle, uint16_t dt_ms,
                    EncoderAngle latest_encoder_value) {
  motion_control_context_t* ctx = &s_contexts[handle];
  EncoderAngle angle_deg = latest_encoder_value; 
    //This if statement might become redundant with new encoders, but we will keep it for now to be safe. We can remove it later if we find that it is not needed.
  if (angle_deg.degree < 10 ||
      angle_deg.degree > (ctx->cfg.encoder.range_of_motion.degree - 10)) {
    // If the encoder is close to its min or max limits, we might be in a
    // situation where the ADC values are off (maybe a loose wire or the
    // encoder is configured incorrectly, for example). In that situation,
    // we want to err on the side of caution and not move the motor.
    stop_motor(handle);
    static int i = 0;
    // Don't print every time to avoid triggering the task watchdog.
    if (--i < 0) {
      i = 50;

      ESP_LOGW(TAG,
               "Encoder angle %f is close to its limits of [0, %f]. "
               "Turning off motor as a safety precaution",
               angle_deg.degree, ctx->cfg.encoder.range_of_motion.degree);
    }
    return;
  }

  // Take snapshot of shared state and update current angle from feedback
  portENTER_CRITICAL(&ctx->spinlock);
  bool estop = ctx->estop_active;
  EncoderAngle target_angle = ctx->target_angle;
  float current_velocity_sps =
      ctx->current_velocity.dps * ctx->steps_per_degree;
  // Aim for the target velocity, but constrain it to the max velocity.
  const AngularVelocity constrained_target_speed_increasing_angle = {
      MIN(ctx->target_velocity.dps, ctx->cfg.max_speed_increasing_angle.dps)};
  const AngularVelocity constrained_target_speed_decreasing_angle = {
      MIN(ctx->target_velocity.dps, ctx->cfg.max_speed_decreasing_angle.dps)};
  ctx->current_angle = angle_deg;
  ctx->use_position_feedback = true;
  portEXIT_CRITICAL(&ctx->spinlock);

  if (estop) {
    stop_motor(handle);
    return;
  }

  //Need to check if we still have a deadband with new encoders
  MotorRampingArgs args = {
      .current_angle = ctx->current_angle,
      .target_angle = ctx->target_angle,
      .deadband = (EncoderAngle){DEADBAND_DEG}, //Change to was PotentiometerAngle check if this is still valid with new encoders
      .current_velocity = ctx->current_velocity,
      .max_acceleration = ctx->cfg.max_accel,
      .max_speed_decreasing_angle = constrained_target_speed_decreasing_angle,
      .max_speed_increasing_angle = constrained_target_speed_increasing_angle,
      .timestep_ms = dt_ms,
  };
  AngularVelocity new_velocity = motor_ramping_trapezoidal(&args);

  // NOTE: Normally checking equality of floats is imprecise, but in this case
  // the literal value 0.F gets returned when within the deadband, so it should
  // be fine checking against the same literal.
  if (new_velocity.dps == 0.F) {
    stop_motor(handle);
    return;
  }

  // Apply motor velocity (handles enable/disable, frequency, duty)
  apply_motor_velocity(handle, new_velocity);

  // Update the current state.
  portENTER_CRITICAL(&ctx->spinlock);
  ctx->current_velocity = new_velocity;
  ctx->is_moving = true;
  portEXIT_CRITICAL(&ctx->spinlock);

  // Logging (periodic, not every update)
  static uint32_t log_counter = 0;
  if (++log_counter >= 100) {  // Log every 100 updates
    log_counter = 0;
    ESP_LOGD(TAG,
             "Update: target=%.2f°, current=%.2f°, error=%.2f°, vel=%.1f sps, "
             "moving=%d",
             target_angle.degree, angle_deg.degree,
             target_angle.degree - angle_deg.degree, current_velocity_sps,
             ctx->is_moving);
  }
}

// ------ Setters ------
//Sets the target angle but does some form of conversion that might not be necessary with new encoders. We will keep it for now to be safe, but we can remove it later if we find that it is not needed.
void stepper_set_target_angle(stepper_control_handle_t handle,
                              JointAngle target_angle) {
  motion_control_context_t* ctx = &s_contexts[handle];

  EncoderAngle target_encoder_angle =
      to_encoder_angle(&ctx->cfg.encoder, target_angle);

  target_encoder_angle = clamp_encoder_angle(
      &ctx->cfg.encoder, target_encoder_angle);
  portENTER_CRITICAL(&ctx->spinlock);
  ctx->target_angle = target_encoder_angle;
  portEXIT_CRITICAL(&ctx->spinlock);
}

void stepper_set_target_velocity(stepper_control_handle_t handle,
                                 AngularVelocity target_velocity) {
  motion_control_context_t* ctx = &s_contexts[handle];
  portENTER_CRITICAL(&ctx->spinlock);
  ctx->target_velocity = target_velocity;
  portEXIT_CRITICAL(&ctx->spinlock);
}

void stepper_set_estop(stepper_control_handle_t handle, bool active) {
  motion_control_context_t* ctx = &s_contexts[handle];

  portENTER_CRITICAL(&ctx->spinlock);
  ctx->estop_active = active;
  portEXIT_CRITICAL(&ctx->spinlock);
  if (active) {
    stop_motor(handle);
  }
}

// ------ Getters ------

EncoderAngle stepper_get_current_angle(stepper_control_handle_t handle) {
  const motion_control_context_t* ctx = &s_contexts[handle];

  portENTER_CRITICAL(&ctx->spinlock);
  EncoderAngle angle = ctx->current_angle;
  portEXIT_CRITICAL(&ctx->spinlock);
  return angle;
}

EncoderAngle stepper_get_target_angle(stepper_control_handle_t handle) {
  const motion_control_context_t* ctx = &s_contexts[handle];

  portENTER_CRITICAL(&ctx->spinlock);
  EncoderAngle angle = ctx->target_angle;
  portEXIT_CRITICAL(&ctx->spinlock);
  return angle;
}

AngularVelocity stepper_get_current_velocity(stepper_control_handle_t handle) {
  const motion_control_context_t* ctx = &s_contexts[handle];

  portENTER_CRITICAL(&ctx->spinlock);
  AngularVelocity velocity = ctx->current_velocity;
  portEXIT_CRITICAL(&ctx->spinlock);
  return velocity;
}

bool stepper_is_moving(stepper_control_handle_t handle) {
  const motion_control_context_t* ctx = &s_contexts[handle];

  portENTER_CRITICAL(&ctx->spinlock);
  bool moving = ctx->is_moving;
  portEXIT_CRITICAL(&ctx->spinlock);
  return moving;
}

bool stepper_has_position_feedback(stepper_control_handle_t handle) {
  const motion_control_context_t* ctx = &s_contexts[handle];

  portENTER_CRITICAL(&ctx->spinlock);
  bool has_feedback = ctx->use_position_feedback;
  portEXIT_CRITICAL(&ctx->spinlock);
  return has_feedback;
}

const stepper_control_config_t* stepper_get_cfg(
    stepper_control_handle_t handle) {
  return &s_contexts[handle].cfg;
}
