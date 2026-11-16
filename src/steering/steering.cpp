#include "steering.h"

#include "../../include/config.h"

// Maps the RC receiver pulse width to the servo pulse width expected by the steering servo.
namespace roadrunner {

uint16_t Steering::pulseFor(uint16_t receiverPulseUs) const {
  // Step 1: clamp the receiver signal to its expected range so invalid values
  // cannot request a steering position beyond the servo's calibrated limits.
  uint16_t input = receiverPulseUs;
  if (input < config::kRcPulseMinUs) {
    input = config::kRcPulseMinUs;
  } else if (input > config::kRcPulseMaxUs) {
    input = config::kRcPulseMaxUs;
  }

  // Step 2: linearly scale the clamped input range to the servo range. Adding
  // half the input range before division rounds to the nearest whole microsecond.
  const uint32_t inputRange = config::kRcPulseMaxUs - config::kRcPulseMinUs;
  const uint32_t outputRange = config::kSteeringMaxUs - config::kSteeringMinUs;
  const uint32_t offset =
      (static_cast<uint32_t>(input - config::kRcPulseMinUs) * outputRange +
       inputRange / 2) /
      inputRange;
  return static_cast<uint16_t>(config::kSteeringMinUs + offset);
}

}  // namespace roadrunner