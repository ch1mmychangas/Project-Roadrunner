#include <Arduino.h>
#include <ESP32Servo.h>

#include "../include/config.h"
#include "../include/pinout.h"
#include "control/vehicle_controller.h"

// Main firmware entry point for the ESP32-based RoadRunner controller.
namespace {

// Receiver channels for throttle, steering, and drive mode.
constexpr uint8_t kThrottleChannel = 0;
constexpr uint8_t kSteeringChannel = 1;
constexpr uint8_t kModeChannel = 2;

// Stores the last measured pulse width and timestamp for each receiver channel.
volatile uint32_t receiverRiseUs[3] = {0, 0, 0};
volatile uint16_t receiverPulseUs[3] = {0, 0, 0};
volatile uint32_t receiverUpdatedAtUs[3] = {0, 0, 0};
portMUX_TYPE receiverMux = portMUX_INITIALIZER_UNLOCKED;

// ESC and steering outputs driven by the controller.
Servo frontEsc;
Servo rearEsc;
Servo steeringServo;
roadrunner::VehicleController vehicleController;
bool outputsReady = false;

// Measures the pulse width on a GPIO and records the latest update time.
void IRAM_ATTR captureReceiverChannel(uint8_t channel, int pin) {
  const uint32_t nowUs = micros();
  const bool high = digitalRead(pin) == HIGH;

  portENTER_CRITICAL_ISR(&receiverMux);
  if (high) {
    // A rising edge starts a new receiver pulse; the falling edge ends it.
    receiverRiseUs[channel] = nowUs;
  } else {
    const uint32_t widthUs = nowUs - receiverRiseUs[channel];
    // Ignore an edge with no recorded start, and reject widths that cannot fit
    // in the pulse-width field. A valid falling edge also refreshes this channel.
    if (receiverRiseUs[channel] != 0 && widthUs <= UINT16_MAX) {
      receiverPulseUs[channel] = static_cast<uint16_t>(widthUs);
      receiverUpdatedAtUs[channel] = nowUs;
    }
  }
  portEXIT_CRITICAL_ISR(&receiverMux);
}

// Interrupt handlers for the three receiver channels.
void IRAM_ATTR captureThrottle() {
  captureReceiverChannel(kThrottleChannel, roadrunner::pins::kReceiverThrottle);
}

void IRAM_ATTR captureSteering() {
  captureReceiverChannel(kSteeringChannel, roadrunner::pins::kReceiverSteering);
}

void IRAM_ATTR captureDriveMode() {
  captureReceiverChannel(kModeChannel, roadrunner::pins::kReceiverDriveMode);
}

// Copies the latest receiver measurements into a stable frame for the controller.
roadrunner::ReceiverFrame readReceiverFrame() {
  roadrunner::ReceiverFrame frame;
  // The ISR can update these values at any time, so copy all three channels
  // while holding the same lock used by the interrupt handlers.
  portENTER_CRITICAL(&receiverMux);
  frame.throttleUs = receiverPulseUs[kThrottleChannel];
  frame.steeringUs = receiverPulseUs[kSteeringChannel];
  frame.driveModeUs = receiverPulseUs[kModeChannel];
  frame.throttleUpdatedAtUs = receiverUpdatedAtUs[kThrottleChannel];
  frame.steeringUpdatedAtUs = receiverUpdatedAtUs[kSteeringChannel];
  frame.driveModeUpdatedAtUs = receiverUpdatedAtUs[kModeChannel];
  portEXIT_CRITICAL(&receiverMux);
  return frame;
}

// Attaches an output servo to the hardware pin and configures its period.
bool attachOutput(Servo& servo, int pin) {
  servo.setPeriodHertz(50);
  return servo.attach(pin, roadrunner::config::kEscPulseMinUs,
                      roadrunner::config::kEscPulseMaxUs) >= 0;
}

// Commands the ESCs and servo to their neutral values while the vehicle is not active.
void writeNeutralOutputs() {
  if (frontEsc.attached()) {
    frontEsc.writeMicroseconds(roadrunner::config::kEscNeutralUs);
  }
  if (rearEsc.attached()) {
    rearEsc.writeMicroseconds(roadrunner::config::kEscNeutralUs);
  }
  if (steeringServo.attached()) {
    steeringServo.writeMicroseconds(roadrunner::config::kSteeringCenterUs);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);

  // Configure the three receiver signals as inputs before enabling interrupts.
  pinMode(roadrunner::pins::kReceiverThrottle, INPUT);
  pinMode(roadrunner::pins::kReceiverSteering, INPUT);
  pinMode(roadrunner::pins::kReceiverDriveMode, INPUT);

  // Start all output devices, then immediately command neutral/center so the
  // ESCs and steering servo do not receive motion commands during startup.
  const bool frontReady = attachOutput(frontEsc, roadrunner::pins::kFrontEscPpm);
  const bool rearReady = attachOutput(rearEsc, roadrunner::pins::kRearEscPpm);
  const bool steeringReady =
      attachOutput(steeringServo, roadrunner::pins::kSteeringServo);
  outputsReady = frontReady && rearReady && steeringReady;
  writeNeutralOutputs();

  // Measure both edges of each receiver signal to calculate pulse widths.
  attachInterrupt(digitalPinToInterrupt(roadrunner::pins::kReceiverThrottle),
                  captureThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(roadrunner::pins::kReceiverSteering),
                  captureSteering, CHANGE);
  attachInterrupt(digitalPinToInterrupt(roadrunner::pins::kReceiverDriveMode),
                  captureDriveMode, CHANGE);

  // Begin the safety state machine only after the hardware and input paths are set up.
  vehicleController.begin(millis());
  if (!outputsReady) {
    // A partially initialized output set must never be treated as drive-ready.
    vehicleController.setEmergencyStopActive(true);
    Serial.println("ERROR: PWM output initialization failed; vehicle remains disarmed.");
  }
}

void loop() {
  if (!outputsReady) {
    // Keep retrying neutral commands and skip all normal control updates.
    writeNeutralOutputs();
    delay(10);
    return;
  }

  // Each loop reads one consistent receiver snapshot and lets the controller
  // apply validation, freshness checks, safety rules, and command mapping.
  const uint32_t nowUs = micros();
  const roadrunner::VehicleOutputs outputs =
      vehicleController.update(readReceiverFrame(), nowUs);

  // Send the controller's final pulse widths to the two ESCs and steering servo.
  frontEsc.writeMicroseconds(outputs.frontEscUs);
  rearEsc.writeMicroseconds(outputs.rearEscUs);
  steeringServo.writeMicroseconds(outputs.steeringUs);
  // Yield briefly so the loop does not continuously poll at full CPU speed.
  delay(5);
}