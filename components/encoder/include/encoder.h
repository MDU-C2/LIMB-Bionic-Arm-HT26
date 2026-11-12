#pragma once

#include <stdbool.h>
#include <stdint.h>

// These are angles from the encoder's frame of reference.
typedef struct {
  float degree;
} EncoderAngle;

// These are angles from the joint's frame of reference.
typedef struct {
  float degree;
} JointAngle;

// This represents the characteristics of the encoder used.
typedef struct {
  // The range of motion that the encoder supports.
  EncoderAngle range_of_motion;

  // The range of motion that the joint supports in its own frame of reference.
  JointAngle min_encoder_angle_as_joint_angle;

  // The range of motion that the joint supports in values expressed by the
  // encoder.
  EncoderAngle min_encoder_angle;
  EncoderAngle max_encoder_angle;

  // The ratio between 1 degree in the joint angle and the corresponding degree
  // in encoder angle.
  float joint_angle_to_encoder_angle_ratio;

  // If you want the relationship between joint angles and
  // encoder angles to be reversed, i.e. the min joint angle
  // corresponds to the max encoder angle and an increase in the
  // joint angle corresponds to a decrease in the encoder angle.
  bool is_reversed;
} Encoder;

// Converts an angle from the encoder's frame of reference to the joint's
// frame of reference.
JointAngle to_joint_angle(const Encoder* encoder,
                          EncoderAngle angle);

// Converts an angle from the joint's frame of reference to the encoder's
// frame of reference.
EncoderAngle to_encoder_angle(const Encoder* encoder,
                                          JointAngle angle);

// Clamps a encoder angle to the limits determined by the encoder.
EncoderAngle clamp_encoder_angle(const Encoder* encoder,
                                             EncoderAngle angle);
