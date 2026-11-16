#pragma once

#include <stdint.h>

#include "../motor/esc_controller.h"
#include "../safety/safety.h"
#include "../steering/steering.h"

// High-level vehicle control logic that combines the receiver inputs, drivetrain rules,
// safety logic, and steering/ESC command generation.
namespace roadrunner {

// Snapshot of the latest receiver pulses and their timestamps.
struct ReceiverFrame {
  uint16_t throttleUs;
  uint16_t steeringUs;
  uint16_t driveModeUs;
  uint32_t throttleUpdatedAtUs;
  uint32_t steeringUpdatedAtUs;
  uint32_t driveModeUpdatedAtUs;
};

// Final pulse commands sent to the front ESC, rear ESC, and steering servo.
struct VehicleOutputs {
  uint16_t frontEscUs;
  uint16_t rearEscUs;
  uint16_t steeringUs;
  bool armed;
};

class VehicleController {
 public:
  VehicleController();

  // Initializes the safety state machine at the given system time.
  void begin(uint32_t nowMs);

  // Produces the output pulses from a fresh receiver frame, while enforcing safety rules.
  VehicleOutputs update(const ReceiverFrame& receiver, uint32_t nowUs);

  // Enables or disables the emergency-stop safety path.
  void setEmergencyStopActive(bool active);

  // Returns the current safety state for monitoring and diagnostics.
  SafetyState safetyState() const;

 private:
  Safety safety_;
  Drivetrain drivetrain_;
  Steering steering_;
};

}  // namespace roadrunner