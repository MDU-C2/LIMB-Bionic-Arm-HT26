#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
  ELBOW_FEEDBACK_SPI_MODE = 0,
  ELBOW_FEEDBACK_SPI_CLOCK_HZ = 500000,
  ELBOW_FEEDBACK_RESOLUTION_BITS = 12,
  ELBOW_FEEDBACK_COUNTS_PER_TURN = 1U << ELBOW_FEEDBACK_RESOLUTION_BITS,
  ELBOW_FEEDBACK_FRAME_SIZE = 4,
  ELBOW_FEEDBACK_FRAME_MARKER = 0xA1,
  ELBOW_FEEDBACK_TIMEOUT_MS = 50,
};

// CRC-8, polynomial 0x07, initial value 0, MSB first.
static inline uint8_t elbow_feedback_crc(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (uint8_t)((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
  }
  return crc;
}

static inline void elbow_feedback_encode(uint8_t* frame, uint16_t counts) {
  frame[0] = ELBOW_FEEDBACK_FRAME_MARKER;
  frame[1] = (uint8_t)(counts >> 8);
  frame[2] = (uint8_t)counts;
  frame[3] = elbow_feedback_crc(frame, ELBOW_FEEDBACK_FRAME_SIZE - 1);
}

static inline bool elbow_feedback_decode(const uint8_t* frame, uint16_t* counts) {
  const uint16_t value = ((uint16_t)frame[1] << 8) | frame[2];
  if (frame[0] != ELBOW_FEEDBACK_FRAME_MARKER ||
      value >= ELBOW_FEEDBACK_COUNTS_PER_TURN ||
      frame[3] != elbow_feedback_crc(frame, ELBOW_FEEDBACK_FRAME_SIZE - 1)) {
    return false;
  }
  *counts = value;
  return true;
}

static inline float elbow_feedback_degrees(uint16_t counts) {
  return counts * (360.0f / ELBOW_FEEDBACK_COUNTS_PER_TURN);
}
