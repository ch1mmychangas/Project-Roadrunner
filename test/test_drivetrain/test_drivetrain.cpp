#include <unity.h>

#include "../../src/drivetrain/drivetrain.h"

// Covers the core drivetrain command logic for all drive modes and braking behavior.
using roadrunner::DriveMode;
using roadrunner::Drivetrain;

// AWD should drive both axles equally when the throttle command is positive.
void test_awd_drives_both_axles() {
  const roadrunner::DriveCommands commands =
      Drivetrain().commandsFor(0.6f, DriveMode::AWD);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, commands.front.drive);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, commands.rear.drive);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, commands.front.brake);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, commands.rear.brake);
}

// Front-dig mode should drive the front axle while braking the rear axle.
void test_front_dig_drives_front_and_brakes_rear() {
  const roadrunner::DriveCommands commands =
      Drivetrain().commandsFor(0.6f, DriveMode::FrontDig);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, commands.front.drive);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, commands.rear.brake);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, commands.front.brake);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, commands.rear.drive);
}

// Rear-dig mode should drive the rear axle while braking the front axle.
void test_rear_dig_drives_rear_and_brakes_front() {
  const roadrunner::DriveCommands commands =
      Drivetrain().commandsFor(0.6f, DriveMode::RearDig);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, commands.rear.drive);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.6f, commands.front.brake);
}

// Negative throttle should generate regenerative-style braking on both axles.
void test_negative_input_brakes_both_axles() {
  const roadrunner::DriveCommands commands =
      Drivetrain().commandsFor(-0.4f, DriveMode::FrontDig);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, commands.front.brake);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, commands.rear.brake);
}

// Unknown modes should fail safely to neutral outputs.
void test_invalid_mode_returns_neutral_commands() {
  const roadrunner::DriveCommands commands =
      Drivetrain().commandsFor(0.8f, static_cast<DriveMode>(255));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, commands.front.drive);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, commands.front.brake);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, commands.rear.drive);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, commands.rear.brake);
}

void setup() {
  delay(1000);
  UNITY_BEGIN();
  RUN_TEST(test_awd_drives_both_axles);
  RUN_TEST(test_front_dig_drives_front_and_brakes_rear);
  RUN_TEST(test_rear_dig_drives_rear_and_brakes_front);
  RUN_TEST(test_negative_input_brakes_both_axles);
  RUN_TEST(test_invalid_mode_returns_neutral_commands);
  UNITY_END();
}

void loop() {}
