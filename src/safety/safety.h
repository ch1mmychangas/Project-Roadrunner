#pragma once

#include <stdint.h>

// Safety state machine for startup, arming, failsafe, and emergency-stop logic.
namespace roadrunner {

enum class SafetyState {
  Disarmed,
  StartupDelay,
  Arming,
  Armed,
  ReceiverFailsafe,
  EmergencyStop
};

class Safety {
 public:
  Safety();

  // Resets the safety controller and starts the startup delay timer.
  void begin(uint32_t nowMs);

  // Updates the safety state from the current system time and receiver health.
  bool update(uint32_t nowMs, bool receiverHealthy, bool throttleNeutral);

  // Enables or disables the hardware emergency-stop path.
  void setEmergencyStopActive(bool active);

  // Returns the current safety state for external monitoring.
  SafetyState state() const;

 private:
  SafetyState state_;
  uint32_t bootTimeMs_;
  uint32_t neutralSinceMs_;
  bool initialized_;
  bool neutralTiming_;
  bool emergencyStopActive_;
};

}  // namespace roadrunner