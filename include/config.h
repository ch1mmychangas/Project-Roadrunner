#pragma once

#include <stdint.h>

// Shared tuning values for the RoadRunner controller.
namespace roadrunner {
namespace config {

// Standard 20 ms PWM frame used by many RC and ESC systems.
constexpr uint32_t kPwmPeriodUs = 20000;

// Receiver pulse limits and deadband for throttle commands.
constexpr uint16_t kRcPulseMinUs = 1000;
constexpr uint16_t kRcPulseMaxUs = 2000;
constexpr uint16_t kRcNeutralUs = 1500;
constexpr uint16_t kThrottleDeadbandUs = 25;
constexpr uint32_t kReceiverTimeoutMs = 100;

// Timing used during startup and neutral arming before motion is allowed.
constexpr uint32_t kStartupDelayMs = 2000;
constexpr uint32_t kNeutralArmTimeMs = 1000;

// ESC calibration range for the drive motor.
constexpr uint16_t kEscPulseMinUs = 1000;
constexpr uint16_t kEscPulseMaxUs = 2000;
constexpr uint16_t kEscNeutralUs = 1500;
constexpr uint16_t kEscDeadbandUs = 25;

// Steering servo limits and center position for the front axle.
constexpr uint16_t kSteeringCenterUs = 1500;
constexpr uint16_t kSteeringMinUs = 1150;
constexpr uint16_t kSteeringMaxUs = 1850;

// Maximum steering or mode outputs for different drivetrain configurations.
constexpr uint16_t kModeAwdMaxUs = 1300;
constexpr uint16_t kModeFrontDigMaxUs = 1650;

// Verify the configured pulse timings remain valid and physically sensible.
static_assert(kEscPulseMinUs < kEscNeutralUs, "ESC minimum must be below neutral");
static_assert(kEscNeutralUs < kEscPulseMaxUs, "ESC maximum must be above neutral");
static_assert(kSteeringMinUs < kSteeringCenterUs, "Steering minimum must be below center");
static_assert(kSteeringCenterUs < kSteeringMaxUs, "Steering maximum must be above center");

}  // namespace config
}  // namespace roadrunner