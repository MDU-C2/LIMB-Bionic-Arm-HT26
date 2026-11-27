#include "open_loop_stepper.h"

#include <math.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_ramping.h"

static const char* const TAG = "Open-loop shoulder";

enum {
  RMT_CLOCK_HZ = 1000000,
  STEP_HIGH_US = 5,
  STEP_LOW_MIN_US = 5,
  // Leave 1 ms for task and transmit overhead between successive bursts.
  BURST_DURATION_US = (OPEN_LOOP_UPDATE_PERIOD_MS - 1) * 1000,
  MAX_BURST_STEPS = BURST_DURATION_US / (STEP_HIGH_US + STEP_LOW_MIN_US),
  BURST_TIMEOUT_MS = 2 * OPEN_LOOP_UPDATE_PERIOD_MS,
};

typedef struct {
  JointAngle target_angle;
  AngularVelocity speed;
  bool stop_requested;
  bool rearm_requested;
} OpenLoopCommand;

typedef struct {
  const stepper_control_config_t* config;
  rmt_channel_handle_t tx;
  rmt_encoder_handle_t pulse_encoder;
  pcnt_unit_handle_t counter;
  rmt_symbol_word_t pulses[MAX_BURST_STEPS];
  JointAngle starting_angle;
  float steps_per_degree;
  int min_steps;
  int max_steps;
  int position_steps;
  int burst_steps;
  TickType_t burst_start;
  AngularVelocity encoder_velocity;
  float fractional_steps;
  bool disabled;
  bool restartable;
} OpenLoopAxis;

static OpenLoopAxis s_axes[kShoulderAxisCount];
static OpenLoopCommand s_commands[kShoulderAxisCount];
static portMUX_TYPE s_command_lock = portMUX_INITIALIZER_UNLOCKED;

// Configure STEP and DIR, then loop the electrical STEP signal into PCNT.
void open_loop_stepper_init(ShoulderAxis index,
                            const stepper_control_config_t* config,
                            JointAngle starting_angle) {
  OpenLoopAxis* axis = &s_axes[index];
  axis->config = config;
  axis->starting_angle = starting_angle;
  const unsigned microsteps = MAX((unsigned)config->microstepping_mode, 1U);
  axis->steps_per_degree =
      config->steps_per_rev * microsteps * config->gear_ratio / 360.0f;
  const Encoder* encoder = &config->encoder;
  const float first_limit = to_joint_angle(encoder, encoder->min_encoder_angle).degree;
  const float second_limit = to_joint_angle(encoder, encoder->max_encoder_angle).degree;
  if (!isfinite(first_limit) || !isfinite(second_limit) ||
      !isfinite(axis->steps_per_degree) || axis->steps_per_degree <= 0.0f ||
      !isfinite(starting_angle.degree) ||
      encoder->joint_angle_to_encoder_angle_ratio <= 0.0f) {
    ESP_LOGE(TAG, "Invalid calibration or starting angle for axis %d", index);
    abort();
  }
  axis->min_steps = (int)ceilf((fminf(first_limit, second_limit) -
                                starting_angle.degree) * axis->steps_per_degree);
  axis->max_steps = (int)floorf((fmaxf(first_limit, second_limit) -
                                 starting_angle.degree) * axis->steps_per_degree);
  if (axis->min_steps > 0 || axis->max_steps < 0) {
    ESP_LOGE(TAG, "Starting angle for axis %d is outside its joint limits", index);
    abort();
  }
  s_commands[index].target_angle = starting_angle;

  ESP_ERROR_CHECK(gpio_set_level(config->step_gpio, 0));
  ESP_ERROR_CHECK(gpio_set_level(config->dir_gpio, 0));
  uint64_t output_pins = (1ULL << config->step_gpio) | (1ULL << config->dir_gpio);
  if (config->enable_gpio != GPIO_NUM_NC) {
    ESP_ERROR_CHECK(gpio_set_level(config->enable_gpio, 1));
    output_pins |= 1ULL << config->enable_gpio;
  }
  const gpio_config_t output_config = {
      .pin_bit_mask = output_pins,
      .mode = GPIO_MODE_OUTPUT,
  };
  ESP_ERROR_CHECK(gpio_config(&output_config));

  const rmt_tx_channel_config_t tx_config = {
      .gpio_num = config->step_gpio,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RMT_CLOCK_HZ,
      .mem_block_symbols = 128,
      .trans_queue_depth = 1,
  };
  const rmt_copy_encoder_config_t encoder_config = {};
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &axis->tx));
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &axis->pulse_encoder));
  ESP_ERROR_CHECK(rmt_enable(axis->tx));

  const pcnt_unit_config_t counter_config = {.low_limit = -1, .high_limit = 30000};
  const pcnt_chan_config_t step_config = {
      .edge_gpio_num = config->step_gpio,
      .level_gpio_num = GPIO_NUM_NC,
  };
  pcnt_channel_handle_t step_input;
  ESP_ERROR_CHECK(pcnt_new_unit(&counter_config, &axis->counter));
  ESP_ERROR_CHECK(pcnt_new_channel(axis->counter, &step_config, &step_input));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(step_input,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
  ESP_ERROR_CHECK(pcnt_unit_enable(axis->counter));
  ESP_ERROR_CHECK(pcnt_unit_start(axis->counter));
}

// A new explicit target command rearms an axis stopped by a CAN STOP command.
void open_loop_stepper_set_target(ShoulderAxis index, JointAngle angle,
                                  AngularVelocity speed) {
  if (!isfinite(angle.degree) || !isfinite(speed.dps)) {
    return;
  }
  portENTER_CRITICAL(&s_command_lock);
  s_commands[index].target_angle = angle;
  s_commands[index].speed = speed;
  s_commands[index].stop_requested = false;
  s_commands[index].rearm_requested = true;
  portEXIT_CRITICAL(&s_command_lock);
}

// Ask the motor task to stop this axis and discard future motion commands.
void open_loop_stepper_request_stop(ShoulderAxis index) {
  portENTER_CRITICAL(&s_command_lock);
  s_commands[index].stop_requested = true;
  s_commands[index].rearm_requested = false;
  portEXIT_CRITICAL(&s_command_lock);
}

// Return the estimated joint angle from the reference plus signed STEP edges.
JointAngle open_loop_stepper_get_angle(ShoulderAxis index) {
  const OpenLoopAxis* axis = &s_axes[index];
  return (JointAngle){axis->starting_angle.degree +
                      (float)axis->position_steps / axis->steps_per_degree};
}

// Account for edges emitted by a completed or interrupted transmission.
static esp_err_t record_burst(OpenLoopAxis* axis, bool completed) {
  int count;
  esp_err_t err = pcnt_unit_get_count(axis->counter, &count);
  if (err != ESP_OK) {
    return err;
  }
  axis->position_steps += axis->burst_steps < 0 ? -count : count;
  const int expected = abs(axis->burst_steps);
  axis->burst_steps = 0;
  return count > expected || (completed && count != expected)
             ? ESP_ERR_INVALID_RESPONSE : ESP_OK;
}

// Stop the waveform before counting any edges that escaped an interrupted burst.
static void disable_axis(OpenLoopAxis* axis, esp_err_t reason) {
  axis->disabled = true;
  axis->restartable = reason == ESP_OK;
  if (axis->config->enable_gpio != GPIO_NUM_NC) {
    ESP_ERROR_CHECK(gpio_set_level(axis->config->enable_gpio, 1));
  }
  esp_err_t stop_error = rmt_disable(axis->tx);
  if (stop_error == ESP_ERR_INVALID_STATE) {
    // TX may finish between the SDK's checks for running and idle states.
    stop_error = rmt_disable(axis->tx);
  }
  if (stop_error != ESP_OK) {
    ESP_LOGE(TAG, "Could not disable STEP GPIO %d: %s", axis->config->step_gpio,
             esp_err_to_name(stop_error));
    return;
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(axis->config->step_gpio, 0));
  if (axis->burst_steps != 0 && record_burst(axis, false) != ESP_OK) {
    ESP_LOGE(TAG, "Could not verify STEP count on GPIO %d", axis->config->step_gpio);
  }
  if (reason != ESP_OK) {
    ESP_LOGE(TAG, "STEP GPIO %d stopped: %s", axis->config->step_gpio,
             esp_err_to_name(reason));
  }
}

// Ramp toward a quantized joint target and retain sub-step movement between ticks.
static int plan_burst(OpenLoopAxis* axis, JointAngle current,
                       const OpenLoopCommand* command) {
  const float min_angle = axis->starting_angle.degree +
      (float)axis->min_steps / axis->steps_per_degree;
  const float max_angle = axis->starting_angle.degree +
      (float)axis->max_steps / axis->steps_per_degree;
  const float target_angle = LIMB_CLAMP(command->target_angle.degree, min_angle, max_angle);
  const int target_steps = (int)lroundf((target_angle - axis->starting_angle.degree) *
                                        axis->steps_per_degree);
  const int remaining = target_steps - axis->position_steps;
  if (remaining == 0 || command->speed.dps <= 0.0f) {
    axis->encoder_velocity.dps = 0.0f;
    axis->fractional_steps = 0.0f;
    return 0;
  }

  const stepper_control_config_t* config = axis->config;
  const Encoder* encoder = &config->encoder;
  const float encoder_ratio = encoder->joint_angle_to_encoder_angle_ratio;
  const MotorRampingArgs args = {
      .current_angle = to_encoder_angle(encoder, current),
      .target_angle = to_encoder_angle(encoder, (JointAngle){target_angle}),
      .current_velocity = axis->encoder_velocity,
      .max_acceleration = {config->max_accel.dps2 * encoder_ratio},
      .max_speed_decreasing_angle = {fminf(command->speed.dps,
                                          config->max_speed_decreasing_angle.dps) * encoder_ratio},
      .max_speed_increasing_angle = {fminf(command->speed.dps,
                                          config->max_speed_increasing_angle.dps) * encoder_ratio},
      .timestep_ms = OPEN_LOOP_UPDATE_PERIOD_MS,
  };
  axis->encoder_velocity = motor_ramping_trapezoidal(&args);
  const float encoder_sign = encoder->is_reversed ? -1.0f : 1.0f;
  const float joint_speed = axis->encoder_velocity.dps * encoder_sign / encoder_ratio;
  if ((joint_speed < 0.0f) != (remaining < 0)) {
    axis->fractional_steps = 0.0f;
    return 0;
  }
  const float budget = axis->fractional_steps + fabsf(joint_speed) *
      axis->steps_per_degree * (OPEN_LOOP_UPDATE_PERIOD_MS / 1000.0f);
  const int count = (int)fminf(budget, MAX_BURST_STEPS);
  axis->fractional_steps = budget - floorf(budget);
  const int bounded_count = MIN(count, abs(remaining));
  return remaining < 0 ? -bounded_count : bounded_count;
}

// Submit one finite waveform using a buffer owned by this axis until TX completes.
static esp_err_t start_burst(OpenLoopAxis* axis, int steps) {
  const int count = abs(steps);
  const int period_us = BURST_DURATION_US / count;
  const rmt_symbol_word_t pulse = {
      .level0 = 1, .duration0 = STEP_HIGH_US,
      .level1 = 0, .duration1 = period_us - STEP_HIGH_US,
  };
  for (int i = 0; i < count; ++i) {
    axis->pulses[i] = pulse;
  }
  const stepper_control_config_t* config = axis->config;
  const bool encoder_decreasing = (steps < 0) != config->encoder.is_reversed;
  const bool dir_high = encoder_decreasing != (config->direction == STEPPER_DIR_REVERSE);
  ESP_ERROR_CHECK(gpio_set_level(config->dir_gpio, dir_high));
  if (config->enable_gpio != GPIO_NUM_NC) {
    ESP_ERROR_CHECK(gpio_set_level(config->enable_gpio, 0));
  }
  esp_rom_delay_us(5);
  ESP_ERROR_CHECK(pcnt_unit_clear_count(axis->counter));
  const rmt_transmit_config_t tx_config = {.flags.queue_nonblocking = true};
  esp_err_t err = rmt_transmit(axis->tx, axis->pulse_encoder, axis->pulses,
                              count * sizeof(pulse), &tx_config);
  if (err == ESP_OK) {
    axis->burst_steps = steps;
    axis->burst_start = xTaskGetTickCount();
  }
  return err;
}

// Poll completion without blocking the other axes and apply the latest command.
void open_loop_stepper_update(ShoulderAxis index) {
  OpenLoopAxis* axis = &s_axes[index];
  portENTER_CRITICAL(&s_command_lock);
  const OpenLoopCommand command = s_commands[index];
  s_commands[index].rearm_requested = false;
  portEXIT_CRITICAL(&s_command_lock);

  if (axis->disabled) {
    if (!axis->restartable || !command.rearm_requested ||
        command.stop_requested) {
      return;
    }
    esp_err_t err = rmt_enable(axis->tx);
    if (err != ESP_OK) {
      axis->restartable = false;
      ESP_LOGE(TAG, "Could not rearm STEP GPIO %d: %s",
               axis->config->step_gpio, esp_err_to_name(err));
      return;
    }
    axis->disabled = false;
    axis->restartable = false;
    axis->encoder_velocity = (AngularVelocity){0.0f};
    axis->fractional_steps = 0.0f;
  }
  if (command.stop_requested) {
    disable_axis(axis, ESP_OK);
    return;
  }

  if (axis->burst_steps != 0) {
    esp_err_t err = rmt_tx_wait_all_done(axis->tx, 0);
    if (err == ESP_ERR_TIMEOUT && xTaskGetTickCount() - axis->burst_start <
                                     pdMS_TO_TICKS(BURST_TIMEOUT_MS)) {
      return;
    }
    if (err == ESP_OK) {
      err = record_burst(axis, true);
    }
    if (err != ESP_OK) {
      disable_axis(axis, err);
      return;
    }
  }

  const int steps = plan_burst(axis, open_loop_stepper_get_angle(index), &command);
  if (steps != 0) {
    const esp_err_t err = start_burst(axis, steps);
    if (err != ESP_OK) {
      disable_axis(axis, err);
    }
  }
}
