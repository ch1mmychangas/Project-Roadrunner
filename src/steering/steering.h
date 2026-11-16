#pragma once

#include <stdint.h>

// Converts a receiver pulse width into a steering servo pulse width.
namespace roadrunner {

class Steering {
 public:
  // Maps the RC pulse range to the steering servo pulse range while clamping input.
  uint16_t pulseFor(uint16_t receiverPulseUs) const;
};

}  // namespace roadrunner