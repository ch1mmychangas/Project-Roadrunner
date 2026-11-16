#include "steering.h"

#include "../../include/config.h"

// Maps the RC receiver pulse width to the servo pulse width expected by the steering servo.
namespace roadrunner {

uint16_t Steering::pulseFor(uint16_t receiverPulseUs) const {
  // Clamp the incoming RC pulse so out-of-range values do not break the servo mapping.
  uint16_t input = receiverPulseUs;
  if (input < config::kRcPulseMinUs) {
    input = config::kRcPulseMinUs;
  } else if (input > config::kRcPulseMaxUs) {
    input = config::kRcPulseMaxUs;
  }

  // Scale the trimmed receiver range into the steering servo range.
  const uint32_t inputRange = config::kRcPulseMaxUs - config::kRcPulseMinUs;
  const uint32_t outputRange = config::kSteeringMaxUs - config::kSteeringMinUs;
  const uint32_t offset =
      (static_cast<uint32_t>(input - config::kRcPulseMinUs) * outputRange +
       inputRange / 2) /
      inputRange;
  return static_cast<uint16_t>(config::kSteeringMinUs + offset);
}

}  // namespace roadrunner