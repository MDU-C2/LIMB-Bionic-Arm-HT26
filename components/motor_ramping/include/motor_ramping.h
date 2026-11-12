#pragma once

#include <stdint.h>

#include "limb_utils.h"
#include "encoder.h"

typedef struct {
  EncoderAngle current_angle;
  EncoderAngle target_angle;
  // The deadband extends +-(deadband / 2) degrees from target_angle.
  EncoderAngle deadband;
  AngularVelocity current_velocity;
  AngularAcceleration max_acceleration;
  // The max speed that the motor should be allowed to move at in the direction
  // that decreases the encoder angle.
  AngularVelocity max_speed_decreasing_angle;
  // The max speed that the motor should be allowed to move at in the direction
  // that increases the encoder angle.
  AngularVelocity max_speed_increasing_angle;
  // The amount of time between updates to the motor state.
  uint32_t timestep_ms;
} MotorRampingArgs;

// Calculates the velocity the motor should use to achieve a trapezoidal ramping
// effect.
AngularVelocity motor_ramping_trapezoidal(const MotorRampingArgs* args);
