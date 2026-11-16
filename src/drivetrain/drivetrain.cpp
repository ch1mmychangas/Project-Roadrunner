#include "drivetrain.h"

#include <math.h>

// Converts a normalized throttle command into axle-specific drive/brake instructions.
namespace roadrunner {
namespace {

// Clamps the raw input to a safe [-1, 1] range and rejects invalid floating-point values.
float clampSigned(float value) {
  if (!isfinite(value)) {
    return 0.0f;
  }
  if (value < -1.0f) {
    return -1.0f;
  }
  return value > 1.0f ? 1.0f : value;
}

// Creates a command that applies forward drive power.
MotorCommand drive(float amount) {
  return {amount, 0.0f};
}

// Creates a command that applies braking force without accelerating the motor.
MotorCommand brake(float amount) {
  return {0.0f, amount};
}

}  // namespace

// For negative throttle, the vehicle brakes both axles symmetrically.
// Positive throttle dispatches according to the selected drivetrain mode.
DriveCommands Drivetrain::commandsFor(float throttle, DriveMode mode) const {
  const float input = clampSigned(throttle);
  // Negative throttle means braking regardless of selected drive mode.
  if (input < 0.0f) {
    const MotorCommand braking = brake(-input);
    return {braking, braking};
  }

  // For forward throttle, the selected mode decides which axle drives and
  // which axle brakes: AWD drives both; each dig mode brakes the opposite axle.
  switch (mode) {
    case DriveMode::FrontDig:
      return {drive(input), brake(input)};
    case DriveMode::RearDig:
      return {brake(input), drive(input)};
    case DriveMode::AWD:
      return {drive(input), drive(input)};
    default:
      return {brake(0.0f), brake(0.0f)};
  }
}

}  // namespace roadrunner