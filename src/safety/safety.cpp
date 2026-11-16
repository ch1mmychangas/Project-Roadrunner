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
    begin(nowMs);
    return false;
  }

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
    return true;
  }

  if (static_cast<uint32_t>(nowMs - bootTimeMs_) < config::kStartupDelayMs) {
    state_ = SafetyState::StartupDelay;
    neutralTiming_ = false;
    return false;
  }

  if (!throttleNeutral) {
    state_ = SafetyState::Disarmed;
    neutralTiming_ = false;
    return false;
  }

  if (!neutralTiming_) {
    neutralSinceMs_ = nowMs;
    neutralTiming_ = true;
    state_ = SafetyState::Arming;
    return false;
  }

  if (static_cast<uint32_t>(nowMs - neutralSinceMs_) < config::kNeutralArmTimeMs) {
    state_ = SafetyState::Arming;
    return false;
  }

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