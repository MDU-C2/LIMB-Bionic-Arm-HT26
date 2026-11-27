#include "open_loop_stepper.h"

#include <math.h>
#include <stdlib.h>

#include "driver/gpio.h"
#if SOC_PCNT_SUPPORTED
#include "driver/pulse_cnt.h"
#endif
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_ramping.h"

static const char* const TAG = "Open-loop stepper";

enum {
  RMT_CLOCK_HZ = 1000000,
  STEP_HIGH_US = 5,
  STEP_LOW_MIN_US = 5,
  // Leave 1 ms for task and transmit overhead between successive bursts.
  BURST_DURATION_US = (OPEN_LOOP_UPDATE_PERIOD_MS - 1) * 1000,
#if SOC_PCNT_SUPPORTED
  AXIS_COUNT = 3,
  RMT_MEM_SYMBOLS = 128,
  MAX_BURST_STEPS = BURST_DURATION_US / (STEP_HIGH_US + STEP_LOW_MIN_US),
#else
  AXIS_COUNT = 1,
  RMT_MEM_SYMBOLS = 2 * SOC_RMT_MEM_WORDS_PER_CHANNEL,
  // With no PCNT, avoid streaming or RMT loop mode. A single hardware
  // transaction holds all STEP pulses plus the end marker.
  MAX_BURST_STEPS = RMT_MEM_SYMBOLS - 1,
#endif
  COMMAND_TIMEOUT_MS = 500,
  BURST_TIMEOUT_MS = 2 * OPEN_LOOP_UPDATE_PERIOD_MS,
};

typedef struct {
  JointAngle target_angle;
  AngularVelocity speed;
  TickType_t last_command;
  bool armed;
  bool pause_requested;
  bool stop_requested;
} OpenLoopCommand;

typedef struct {
  const stepper_control_config_t* config;
  rmt_channel_handle_t tx;
  rmt_encoder_handle_t pulse_encoder;
#if SOC_PCNT_SUPPORTED
  pcnt_unit_handle_t counter;
#endif
  rmt_symbol_word_t pulses[MAX_BURST_STEPS];
  JointAngle starting_angle;
  float steps_per_degree;
  int min_steps;
  int max_steps;
  int position_steps;
  int burst_steps;
  TickType_t burst_start;
  AngularVelocity joint_velocity;
  float fractional_steps;
  bool joint_direction_reversed;
  bool armed;
  bool disabled;
} OpenLoopAxis;

static OpenLoopAxis s_axes[AXIS_COUNT];
static OpenLoopCommand s_commands[AXIS_COUNT];
static portMUX_TYPE s_command_lock = portMUX_INITIALIZER_UNLOCKED;

void open_loop_stepper_init(unsigned index,
                            const stepper_control_config_t* config,
                            OpenLoopReference reference) {
  OpenLoopAxis* axis = &s_axes[index];
  axis->config = config;
  axis->starting_angle = reference.starting_angle;
  axis->joint_direction_reversed = reference.joint_direction_reversed;
  const unsigned microsteps = MAX((unsigned)config->microstepping_mode, 1U);
  axis->steps_per_degree =
      config->steps_per_rev * microsteps * config->gear_ratio / 360.0f;
  if (!isfinite(reference.min_angle.degree) ||
      !isfinite(reference.max_angle.degree) ||
      reference.min_angle.degree >= reference.max_angle.degree ||
      !isfinite(axis->steps_per_degree) || axis->steps_per_degree <= 0.0f ||
      !isfinite(reference.starting_angle.degree) ||
      reference.starting_angle.degree < reference.min_angle.degree ||
      reference.starting_angle.degree > reference.max_angle.degree) {
    axis->disabled = true;
    ESP_LOGE(TAG, "Axis %u blocked: missing joint limits or invalid reference", index);
    return;
  }
  axis->min_steps = (int)ceilf((reference.min_angle.degree -
                                axis->starting_angle.degree) * axis->steps_per_degree);
  axis->max_steps = (int)floorf((reference.max_angle.degree -
                                 axis->starting_angle.degree) * axis->steps_per_degree);

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
      .mem_block_symbols = RMT_MEM_SYMBOLS,
      .trans_queue_depth = 1,
  };
  const rmt_copy_encoder_config_t encoder_config = {};
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &axis->tx));
  ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &axis->pulse_encoder));
  ESP_ERROR_CHECK(rmt_enable(axis->tx));

#if SOC_PCNT_SUPPORTED
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
#endif
}

static bool command_timed_out(const OpenLoopCommand* command, TickType_t now) {
  return command->speed.dps > 0.0f &&
         now - command->last_command >= pdMS_TO_TICKS(COMMAND_TIMEOUT_MS);
}

void open_loop_stepper_set_target(unsigned index, JointAngle angle,
                                  AngularVelocity speed) {
  if (!isfinite(angle.degree) || !isfinite(speed.dps) || speed.dps < 0.0f) {
    return;
  }
  portENTER_CRITICAL(&s_command_lock);
  OpenLoopCommand* command = &s_commands[index];
  const TickType_t now = xTaskGetTickCount();
  if (command_timed_out(command, now)) {
    command->stop_requested = true;
  }
  if (!command->stop_requested) {
    if (speed.dps == 0.0f && signbit(speed.dps)) {
      if (angle.degree == s_axes[index].starting_angle.degree) {
        command->armed = true;
      }
    } else if (speed.dps == 0.0f) {
      command->speed = speed;
      command->pause_requested = true;
    } else if (command->armed && !command->pause_requested) {
      command->target_angle = angle;
      command->speed = speed;
      command->last_command = now;
    }
  }
  portEXIT_CRITICAL(&s_command_lock);
}

void open_loop_stepper_request_stop(unsigned index) {
  portENTER_CRITICAL(&s_command_lock);
  s_commands[index].stop_requested = true;
  portEXIT_CRITICAL(&s_command_lock);
}

JointAngle open_loop_stepper_get_angle(unsigned index) {
  const OpenLoopAxis* axis = &s_axes[index];
  if (!axis->armed || axis->disabled) {
    return (JointAngle){NAN};
  }
  return (JointAngle){axis->starting_angle.degree +
                      (float)axis->position_steps / axis->steps_per_degree};
}

// Credit a burst only on completion. Shoulder also verifies the electrical
// STEP edges. On ESP32-C3 the finite, memory-resident transaction is the count.
static esp_err_t record_burst(OpenLoopAxis* axis) {
#if SOC_PCNT_SUPPORTED
  int count;
  esp_err_t err = pcnt_unit_get_count(axis->counter, &count);
  if (err != ESP_OK) {
    return err;
  }
  if (count != abs(axis->burst_steps)) {
    return ESP_ERR_INVALID_RESPONSE;
  }
#endif
  axis->position_steps += axis->burst_steps;
  axis->burst_steps = 0;
  return ESP_OK;
}

static void disable_axis(OpenLoopAxis* axis, esp_err_t reason) {
  axis->disabled = true;
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
  ESP_LOGW(TAG, "STEP GPIO %d stopped: %s", axis->config->step_gpio,
           reason == ESP_OK ? "emergency stop" : esp_err_to_name(reason));
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
    axis->joint_velocity.dps = 0.0f;
    axis->fractional_steps = 0.0f;
    return 0;
  }

  const stepper_control_config_t* config = axis->config;
  const MotorRampingArgs args = {
      .current_angle = {current.degree},
      .target_angle = {target_angle},
      .current_velocity = axis->joint_velocity,
      .max_acceleration = config->max_accel,
      .max_speed_decreasing_angle = {fminf(command->speed.dps,
                                          config->max_speed_decreasing_angle.dps)},
      .max_speed_increasing_angle = {fminf(command->speed.dps,
                                          config->max_speed_increasing_angle.dps)},
      .timestep_ms = OPEN_LOOP_UPDATE_PERIOD_MS,
  };
  axis->joint_velocity = motor_ramping_trapezoidal(&args);
  const float joint_speed = axis->joint_velocity.dps;
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
  const bool control_angle_decreasing = (steps < 0) != axis->joint_direction_reversed;
  const bool dir_high = control_angle_decreasing != (config->direction == STEPPER_DIR_REVERSE);
  ESP_ERROR_CHECK(gpio_set_level(config->dir_gpio, dir_high));
  if (config->enable_gpio != GPIO_NUM_NC) {
    ESP_ERROR_CHECK(gpio_set_level(config->enable_gpio, 0));
  }
  esp_rom_delay_us(5);
#if SOC_PCNT_SUPPORTED
  esp_err_t clear_error = pcnt_unit_clear_count(axis->counter);
  if (clear_error != ESP_OK) {
    return clear_error;
  }
#endif
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
void open_loop_stepper_update(unsigned index) {
  OpenLoopAxis* axis = &s_axes[index];
  if (axis->disabled) {
    return;
  }
  portENTER_CRITICAL(&s_command_lock);
  const bool timed_out = command_timed_out(&s_commands[index], xTaskGetTickCount());
  if (timed_out) {
    s_commands[index].stop_requested = true;
  }
  const OpenLoopCommand command = s_commands[index];
  s_commands[index].pause_requested = false;
  portEXIT_CRITICAL(&s_command_lock);
  if (command.stop_requested) {
    disable_axis(axis, timed_out ? ESP_ERR_TIMEOUT : ESP_OK);
    return;
  }
  axis->armed = command.armed;
  if (!axis->armed) {
    return;
  }

  if (command.pause_requested) {
    axis->joint_velocity.dps = 0.0f;
    axis->fractional_steps = 0.0f;
  }
  if (axis->burst_steps != 0) {
    esp_err_t err = rmt_tx_wait_all_done(axis->tx, 0);
    if (err == ESP_ERR_TIMEOUT && xTaskGetTickCount() - axis->burst_start <
                                     pdMS_TO_TICKS(BURST_TIMEOUT_MS)) {
      return;
    }
    if (err == ESP_OK) {
      err = record_burst(axis);
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
