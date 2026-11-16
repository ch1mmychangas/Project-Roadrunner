#include <unity.h>

#include "../../src/safety/safety.h"

// Confirms the arming safety sequence behaves correctly under startup, failsafe, and E-stop conditions.
using roadrunner::Safety;
using roadrunner::SafetyState;

// The controller should wait for the startup period and a neutral throttle before arming.
void test_requires_boot_delay_and_continuous_neutral_to_arm() {
  Safety safety;
  safety.begin(0);
  TEST_ASSERT_FALSE(safety.update(1999, true, true));
  TEST_ASSERT_FALSE(safety.update(2000, true, true));
  TEST_ASSERT_FALSE(safety.update(2999, true, true));
  TEST_ASSERT_TRUE(safety.update(3000, true, true));
  TEST_ASSERT_EQUAL(static_cast<int>(SafetyState::Armed),
                    static_cast<int>(safety.state()));
}

// A failed receiver must trip the failsafe and re-arm only after all safety checks pass again.
void test_receiver_loss_disarms_and_requires_rearming() {
  Safety safety;
  safety.begin(0);
  safety.update(2000, true, true);
  safety.update(3000, true, true);
  TEST_ASSERT_FALSE(safety.update(3100, false, true));
  TEST_ASSERT_EQUAL(static_cast<int>(SafetyState::ReceiverFailsafe),
                    static_cast<int>(safety.state()));
  TEST_ASSERT_FALSE(safety.update(3200, true, true));
  TEST_ASSERT_FALSE(safety.update(4199, true, true));
  TEST_ASSERT_TRUE(safety.update(4200, true, true));
}

// The emergency stop should prevent arming until it is manually reset.
void test_emergency_stop_blocks_arming_until_cleared() {
  Safety safety;
  safety.begin(0);
  safety.setEmergencyStopActive(true);
  TEST_ASSERT_FALSE(safety.update(5000, true, true));
  TEST_ASSERT_EQUAL(static_cast<int>(SafetyState::EmergencyStop),
                    static_cast<int>(safety.state()));
  safety.setEmergencyStopActive(false);
  TEST_ASSERT_FALSE(safety.update(5001, true, true));
}

void setup() {
  delay(1000);
  UNITY_BEGIN();
  RUN_TEST(test_requires_boot_delay_and_continuous_neutral_to_arm);
  RUN_TEST(test_receiver_loss_disarms_and_requires_rearming);
  RUN_TEST(test_emergency_stop_blocks_arming_until_cleared);
  UNITY_END();
}

void loop() {}
