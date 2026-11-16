#include "esc_controller.h"

#include <math.h>

#include "../../include/config.h"

// Converts normalized motor commands into microsecond pulse lengths for each ESC.
namespace roadrunner {
namespace {

// Keeps the command in [0, 1] and rejects invalid values such as NaN or negatives.
float clampUnit(float value) {
  if (!isfinite(value) || value < 0.0f) {
    return 0.0f;
  }
  return value > 1.0f ? 1.0f : value;
}

// Rounds a float into the nearest integer pulse length for the ESC bus.
uint16_t roundedPulse(float value) {
  return static_cast<uint16_t>(value + 0.5f);
}

}  // namespace

uint16_t EscController::pulseFor(const MotorCommand& command) {
  const float drive = clampUnit(command.drive);
  const float brake = clampUnit(command.brake);

  if (drive > 0.0f && brake == 0.0f) {
    const float usableRange =
        config::kEscPulseMaxUs - config::kEscNeutralUs - config::kEscDeadbandUs;
    return roundedPulse(config::kEscNeutralUs + config::kEscDeadbandUs +
                        drive * usableRange);
  }

  if (brake > 0.0f && drive == 0.0f) {
    const float usableRange =
        config::kEscNeutralUs - config::kEscPulseMinUs - config::kEscDeadbandUs;
    return roundedPulse(config::kEscNeutralUs - config::kEscDeadbandUs -
                        brake * usableRange);
  }

  return config::kEscNeutralUs;
}

EscPulseOutputs EscController::pulsesFor(const DriveCommands& commands) {
  return {pulseFor(commands.front), pulseFor(commands.rear)};
}

}  // namespace roadrunner