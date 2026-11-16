#include <unity.h>

#include "../../src/motor/esc_controller.h"

// Exercises the ESC pulse conversion logic for neutral, drive, and brake commands.
using roadrunner::DriveCommands;
using roadrunner::EscController;
using roadrunner::MotorCommand;

// Neutral throttle should remain centered in the ESC range.
void test_neutral_command_maps_to_center_pulse() {
  TEST_ASSERT_EQUAL_UINT16(1500, EscController::pulseFor({0.0f, 0.0f}));
}

// Full-drive and full-brake commands should stay inside the configured limits.
void test_full_drive_and_brake_are_clamped_to_pulse_limits() {
  TEST_ASSERT_EQUAL_UINT16(2000, EscController::pulseFor({1.0f, 0.0f}));
  TEST_ASSERT_EQUAL_UINT16(1000, EscController::pulseFor({0.0f, 1.0f}));
  TEST_ASSERT_EQUAL_UINT16(1500, EscController::pulseFor({2.0f, 2.0f}));
}

// The combined front/rear command helper must generate distinct outputs per axle.
void test_drive_commands_produce_independent_pulses() {
  const DriveCommands commands = {{1.0f, 0.0f}, {0.0f, 1.0f}};
  const roadrunner::EscPulseOutputs outputs = EscController::pulsesFor(commands);
  TEST_ASSERT_EQUAL_UINT16(2000, outputs.frontUs);
  TEST_ASSERT_EQUAL_UINT16(1000, outputs.rearUs);
}

void setup() {
  delay(1000);
  UNITY_BEGIN();
  RUN_TEST(test_neutral_command_maps_to_center_pulse);
  RUN_TEST(test_full_drive_and_brake_are_clamped_to_pulse_limits);
  RUN_TEST(test_drive_commands_produce_independent_pulses);
  UNITY_END();
}

void loop() {}
