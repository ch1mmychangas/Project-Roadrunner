#include <unity.h>

#include "../../src/steering/steering.h"

// Verifies that steering mapping and clamping behave as expected.
using roadrunner::Steering;

// Confirms the steering servo reaches the expected pulse values at the RC endpoints.
void test_steering_maps_center_and_endpoints() {
  const Steering steering;
  TEST_ASSERT_EQUAL_UINT16(1150, steering.pulseFor(1000));
  TEST_ASSERT_EQUAL_UINT16(1500, steering.pulseFor(1500));
  TEST_ASSERT_EQUAL_UINT16(1850, steering.pulseFor(2000));
}

// Ensures out-of-range receiver pulses are clamped before mapping.
void test_steering_clamps_out_of_range_receiver_values() {
  const Steering steering;
  TEST_ASSERT_EQUAL_UINT16(1150, steering.pulseFor(0));
  TEST_ASSERT_EQUAL_UINT16(1850, steering.pulseFor(65535));
}

void setup() {
  delay(1000);
  UNITY_BEGIN();
  RUN_TEST(test_steering_maps_center_and_endpoints);
  RUN_TEST(test_steering_clamps_out_of_range_receiver_values);
  UNITY_END();
}

void loop() {}
