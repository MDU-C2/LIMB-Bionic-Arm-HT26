#include "encoder.h"

#include <stdint.h>
#include <sys/param.h>

#include "limb_utils.h"


JointAngle to_joint_angle(const Encoder* encoder,
                          EncoderAngle angle) {
  const float degrees_from_min_joint_angle =
      encoder->is_reversed
          ? encoder->max_encoder_angle.degree - angle.degree
          : angle.degree - encoder->min_encoder_angle.degree;

  return (JointAngle){
      encoder->min_encoder_angle_as_joint_angle.degree +
      (degrees_from_min_joint_angle /
       encoder->joint_angle_to_encoder_angle_ratio)};
}

EncoderAngle to_encoder_angle(const Encoder* encoder,
                                          JointAngle angle) {
  const float distance_from_min_joint_angle =
      angle.degree -
      encoder->min_encoder_angle_as_joint_angle.degree;
  const float distance_as_encoder_angle =
      distance_from_min_joint_angle *
      encoder->joint_angle_to_encoder_angle_ratio;

  return (EncoderAngle){
      encoder->is_reversed
          ? encoder->max_encoder_angle.degree -
                distance_as_encoder_angle
          : encoder->min_encoder_angle.degree +
                distance_as_encoder_angle};
}

EncoderAngle clamp_encoder_angle(const Encoder* encoder,
                                             EncoderAngle angle) {
  return (EncoderAngle){
      LIMB_CLAMP(angle.degree, encoder->min_encoder_angle.degree,
                 encoder->max_encoder_angle.degree)};
}
