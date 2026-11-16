#include "vehicle_controller.h"

#include "../../include/config.h"

// Validates receiver data, applies timing and safety constraints, and converts commands to ESC/servo pulses.
namespace roadrunner {
namespace {

// Confirms that the pulse width is within the RC receiver's valid operating range.
bool pulseIsValid(uint16_t pulseUs) {
  return pulseUs >= config::kRcPulseMinUs && pulseUs <= config::kRcPulseMaxUs;
}

// Determines whether the receiver pulse is fresh enough to trust.
bool isFresh(uint32_t updatedAtUs, uint32_t nowUs) {
  return static_cast<uint32_t>(nowUs - updatedAtUs) <=
         config::kReceiverTimeoutMs * 1000UL;
}

// Maps a receiver throttle pulse to a normalized command in the [-1, 1] range.
float throttleCommand(uint16_t pulseUs) {
  const uint16_t forwardStart =
      config::kRcNeutralUs + config::kThrottleDeadbandUs;
  const uint16_t brakeStart =
      config::kRcNeutralUs - config::kThrottleDeadbandUs;

  if (pulseUs > forwardStart) {
    return static_cast<float>(pulseUs - forwardStart) /
           (config::kRcPulseMaxUs - forwardStart);
  }
  if (pulseUs < brakeStart) {
    return -static_cast<float>(brakeStart - pulseUs) /
           (brakeStart - config::kRcPulseMinUs);
  }
  return 0.0f;
}

// Selects the drivetrain behavior from the receiver drive-mode pulse.
DriveMode driveMode(uint16_t pulseUs) {
  if (pulseUs <= config::kModeAwdMaxUs) {
    return DriveMode::AWD;
  }
  if (pulseUs <= config::kModeFrontDigMaxUs) {
    return DriveMode::FrontDig;
  }
  return DriveMode::RearDig;
}

// Returns the safe neutral outputs used when the vehicle is disarmed or failsafe.
VehicleOutputs safeOutputs() {
  return {config::kEscNeutralUs, config::kEscNeutralUs,
          config::kSteeringCenterUs, false};
}

}  // namespace

VehicleController::VehicleController() : safety_(), drivetrain_(), steering_() {}

void VehicleController::begin(uint32_t nowMs) {
  safety_.begin(nowMs);
}

VehicleOutputs VehicleController::update(const ReceiverFrame& receiver, uint32_t nowUs) {
  const bool valid =
      pulseIsValid(receiver.throttleUs) && pulseIsValid(receiver.steeringUs) &&
      pulseIsValid(receiver.driveModeUs);
  const bool fresh =
      isFresh(receiver.throttleUpdatedAtUs, nowUs) &&
      isFresh(receiver.steeringUpdatedAtUs, nowUs) &&
      isFresh(receiver.driveModeUpdatedAtUs, nowUs);
  const bool throttleNeutral =
      receiver.throttleUs >= config::kRcNeutralUs - config::kThrottleDeadbandUs &&
      receiver.throttleUs <= config::kRcNeutralUs + config::kThrottleDeadbandUs;
  const bool armed = safety_.update(nowUs / 1000UL, valid && fresh, throttleNeutral);

  if (!armed) {
    return safeOutputs();
  }

  const DriveCommands commands =
      drivetrain_.commandsFor(throttleCommand(receiver.throttleUs),
                               driveMode(receiver.driveModeUs));
  const EscPulseOutputs escPulses = EscController::pulsesFor(commands);
  return {escPulses.frontUs, escPulses.rearUs,
          steering_.pulseFor(receiver.steeringUs), true};
}

void VehicleController::setEmergencyStopActive(bool active) {
  safety_.setEmergencyStopActive(active);
}

SafetyState VehicleController::safetyState() const {
  return safety_.state();
}

}  // namespace roadrunner