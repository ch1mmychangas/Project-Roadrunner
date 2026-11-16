#include "safety.h"

#include "../../include/config.h"

// Enforces startup timing, neutral-state arming, receiver health, and E-stop behavior.
namespace roadrunner {

Safety::Safety()
    : state_(SafetyState::Disarmed),
      bootTimeMs_(0),
      neutralSinceMs_(0),
      initialized_(false),
      neutralTiming_(false),
      emergencyStopActive_(false) {}

void Safety::begin(uint32_t nowMs) {
  bootTimeMs_ = nowMs;
  neutralSinceMs_ = 0;
  initialized_ = true;
  neutralTiming_ = false;
  emergencyStopActive_ = false;
  state_ = SafetyState::StartupDelay;
}

bool Safety::update(uint32_t nowMs, bool receiverHealthy, bool throttleNeutral) {
  if (!initialized_) {
    // Fail closed if update() is called before begin(): start timing, but do not arm.
    begin(nowMs);
    return false;
  }

  // Check immediate stop conditions before considering startup or arming timers.
  if (emergencyStopActive_) {
    state_ = SafetyState::EmergencyStop;
    neutralTiming_ = false;
    return false;
  }

  if (!receiverHealthy) {
    state_ = SafetyState::ReceiverFailsafe;
    neutralTiming_ = false;
    return false;
  }

  if (state_ == SafetyState::Armed) {
    // Once armed, remain armed while receiver health and E-stop checks pass.
    return true;
  }

  // Require the startup delay to finish before accepting the neutral arming sequence.
  if (static_cast<uint32_t>(nowMs - bootTimeMs_) < config::kStartupDelayMs) {
    state_ = SafetyState::StartupDelay;
    neutralTiming_ = false;
    return false;
  }

  if (!throttleNeutral) {
    // Any throttle movement resets the continuous-neutral timer.
    state_ = SafetyState::Disarmed;
    neutralTiming_ = false;
    return false;
  }

  if (!neutralTiming_) {
    // Start measuring how long throttle has continuously stayed neutral.
    neutralSinceMs_ = nowMs;
    neutralTiming_ = true;
    state_ = SafetyState::Arming;
    return false;
  }

  if (static_cast<uint32_t>(nowMs - neutralSinceMs_) < config::kNeutralArmTimeMs) {
    state_ = SafetyState::Arming;
    return false;
  }

  // Both the startup wait and the full neutral interval have now elapsed.
  state_ = SafetyState::Armed;
  return true;
}

void Safety::setEmergencyStopActive(bool active) {
  emergencyStopActive_ = active;
  if (!active && state_ == SafetyState::EmergencyStop) {
    state_ = SafetyState::Disarmed;
    neutralTiming_ = false;
  }
}

SafetyState Safety::state() const {
  return state_;
}

}  // namespace roadrunner