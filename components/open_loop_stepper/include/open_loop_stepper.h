#pragma once

#include "soc/soc_caps.h"
#include "stepper.h"

#if SOC_PCNT_SUPPORTED
enum { OPEN_LOOP_UPDATE_PERIOD_MS = 10 };
#else
// Keep a full speed burst and its end marker in ESP32-C3 RMT memory.
enum { OPEN_LOOP_UPDATE_PERIOD_MS = 5 };
#endif

typedef struct {
  JointAngle starting_angle;
  JointAngle min_angle;
  JointAngle max_angle;
  bool joint_direction_reversed;
} OpenLoopReference;

void open_loop_stepper_init(unsigned index, const stepper_control_config_t* config,
                            OpenLoopReference reference);

// Actuation uses positive speed for motion, +0 for normal stop and -0 at the
// reference angle for explicit arming. Positive-speed commands must be refreshed.
void open_loop_stepper_set_target(unsigned index, JointAngle angle,
                                  AngularVelocity speed);
void open_loop_stepper_request_stop(unsigned index);

// The motor task owns the pulse generator and estimate. Unarmed and faulted
// axes report NaN.
void open_loop_stepper_update(unsigned index);
JointAngle open_loop_stepper_get_angle(unsigned index);
