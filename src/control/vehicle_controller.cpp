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
  // Unsigned subtraction also handles the micros() counter wrapping around.
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
  // Step 1: every channel must be inside the receiver's configured pulse range.
  const bool valid =
      pulseIsValid(receiver.throttleUs) && pulseIsValid(receiver.steeringUs) &&
      pulseIsValid(receiver.driveModeUs);
  // Step 2: every channel must also have produced a recent pulse.
  const bool fresh =
      isFresh(receiver.throttleUpdatedAtUs, nowUs) &&
      isFresh(receiver.steeringUpdatedAtUs, nowUs) &&
      isFresh(receiver.driveModeUpdatedAtUs, nowUs);
  // Step 3: arming is permitted only while throttle remains inside its deadband.
  const bool throttleNeutral =
      receiver.throttleUs >= config::kRcNeutralUs - config::kThrottleDeadbandUs &&
      receiver.throttleUs <= config::kRcNeutralUs + config::kThrottleDeadbandUs;
  // The safety state machine decides whether any motion output may be produced.
  const bool armed = safety_.update(nowUs / 1000UL, valid && fresh, throttleNeutral);

  if (!armed) {
    // Invalid, stale, or not-yet-armed input always produces neutral outputs.
    return safeOutputs();
  }

  // Step 4: turn the throttle pulse into a signed command and select the axle mode.
  const DriveCommands commands =
      drivetrain_.commandsFor(throttleCommand(receiver.throttleUs),
                               driveMode(receiver.driveModeUs));
  // Step 5: convert each axle's drive/brake command into its ESC pulse width.
  const EscPulseOutputs escPulses = EscController::pulsesFor(commands);
  // Step 6: map steering separately, then return the complete output frame.
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