#pragma once

// Board-specific GPIO assignments for the receiver, drive ESCs, and steering servo.
namespace roadrunner {
namespace pins {

// RC receiver inputs: throttle, steering, and drive-mode selection.
constexpr int kReceiverThrottle = 35;
constexpr int kReceiverSteering = 34;
constexpr int kReceiverDriveMode = 32;

// Output pins for the front/rear electronic speed controllers and steering servo.
constexpr int kFrontEscPpm = 18;
constexpr int kRearEscPpm = 19;
constexpr int kSteeringServo = 21;

}  // namespace pins
}  // namespace roadrunner