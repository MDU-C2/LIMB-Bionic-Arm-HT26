#pragma once

#include "stepper.h"

typedef enum {
  kShoulderUpDown,
  kShoulderLeftRight,
  kShoulderUpperArmRotation,
  kShoulderAxisCount,
} ShoulderAxis;

enum { OPEN_LOOP_UPDATE_PERIOD_MS = 10 };

// Give the pulse counter a known joint reference before enabling CAN commands.
void open_loop_stepper_init(ShoulderAxis index,
                            const stepper_control_config_t* config,
                            JointAngle starting_angle);

// These command functions can be called from the CAN receiver task.
void open_loop_stepper_set_target(ShoulderAxis index, JointAngle angle,
                                  AngularVelocity speed);
void open_loop_stepper_request_stop(ShoulderAxis index);

// The motor task owns the pulse generator, counter and position estimate.
void open_loop_stepper_update(ShoulderAxis index);
JointAngle open_loop_stepper_get_angle(ShoulderAxis index);
