#pragma once

#include <stdint.h>

#include "../drivetrain/drivetrain.h"

// Converts drivetrain commands into ESC pulse widths for the front and rear motors.
namespace roadrunner {

struct EscPulseOutputs {
  uint16_t frontUs;
  uint16_t rearUs;
};

class EscController {
 public:
  // Converts a single motor command to a pulse width for a brushless ESC.
  static uint16_t pulseFor(const MotorCommand& command);

  // Converts both motors into their final ESC pulse outputs.
  static EscPulseOutputs pulsesFor(const DriveCommands& commands);
};

}  // namespace roadrunner