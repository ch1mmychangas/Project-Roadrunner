#pragma once

// The drivetrain converts a single throttle input into per-axle drive/brake commands.
namespace roadrunner {

// The controller supports all-wheel drive and front/rear differential modes.
enum class DriveMode {
  AWD,
  FrontDig,
  RearDig
};

// One motor command carries a positive drive value and/or a positive brake value.
struct MotorCommand {
  float drive;
  float brake;
};

// Each axle may be commanded independently for forward motion and braking.
struct DriveCommands {
  MotorCommand front;
  MotorCommand rear;
};

class Drivetrain {
 public:
  // Returns the command mix for the front and rear motors based on throttle and mode.
  DriveCommands commandsFor(float throttle, DriveMode mode) const;
};

}  // namespace roadrunner