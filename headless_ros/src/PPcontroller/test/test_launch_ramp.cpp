#include "launch_ramp.hpp"

#include <gtest/gtest.h>

TEST(LaunchRamp, EngagesOnlyWhenTargetIsSufficientlyHigher) {
  EXPECT_TRUE(launch_ramp::should_engage(2.0, 1.0, 0.5));
  EXPECT_TRUE(launch_ramp::should_engage(1.5, 1.0, 0.5));
  EXPECT_FALSE(launch_ramp::should_engage(1.4, 1.0, 0.5));
  EXPECT_FALSE(launch_ramp::should_engage(1.0, 1.0, 0.0));
  EXPECT_FALSE(launch_ramp::should_engage(0.5, 1.0, 0.1));
}

TEST(LaunchRamp, TargetMustRemainAboveMeasuredSpeed) {
  EXPECT_TRUE(launch_ramp::target_is_above(2.0, 1.99));
  EXPECT_FALSE(launch_ramp::target_is_above(2.0, 2.0));
  EXPECT_FALSE(launch_ramp::target_is_above(2.0, 2.01));
}

TEST(LaunchRamp, LimitsOnlyPositiveAcceleration) {
  const auto result = launch_ramp::advance(1.0, 4.0, 0.06, 0.1);

  EXPECT_NEAR(result.speed, 1.06, 1e-12);
  EXPECT_TRUE(result.active);
  EXPECT_TRUE(result.command_valid);
  EXPECT_FALSE(result.target_not_above_ramp);
}

TEST(LaunchRamp, BoundsDelayedCallbackToOneNominalStep) {
  EXPECT_NEAR(launch_ramp::bounded_max_step(3.0, 0.2, 0.01), 0.03,
              1e-12);
  EXPECT_NEAR(launch_ramp::bounded_max_step(3.0, 0.001, 0.01), 0.003,
              1e-12);
}

TEST(LaunchRamp, LowerTargetReturnsControlToNormalDeceleration) {
  const auto result = launch_ramp::advance(2.0, 1.2, 0.06, 0.1);

  EXPECT_DOUBLE_EQ(result.speed, 2.0);
  EXPECT_FALSE(result.active);
  EXPECT_FALSE(result.command_valid);
  EXPECT_TRUE(result.target_not_above_ramp);
}

TEST(LaunchRamp, ReleaseCycleStillPublishesLastRampCommand) {
  const auto result = launch_ramp::advance(1.90, 2.0, 0.06, 0.05);

  EXPECT_NEAR(result.speed, 1.96, 1e-12);
  EXPECT_FALSE(result.active);
  EXPECT_TRUE(result.command_valid);
  EXPECT_FALSE(result.target_not_above_ramp);
}

TEST(LaunchRamp, ConfiguredReleaseStartsLimitedHandoff) {
  const auto release = launch_ramp::advance(1.74, 1.92, 0.03, 0.16);
  EXPECT_NEAR(release.speed, 1.77, 1e-12);
  EXPECT_FALSE(release.active);
  EXPECT_TRUE(release.command_valid);

  const auto handoff =
      launch_ramp::advance_handoff(release.speed, 1.92, 1.83, 0.03);
  EXPECT_NEAR(handoff.speed, 1.80, 1e-12);
  EXPECT_TRUE(handoff.active);
  EXPECT_TRUE(handoff.command_valid);
}

TEST(LaunchRamp, HandoffLimitsCloselyFollowingCallback) {
  const auto result =
      launch_ramp::advance_handoff(1.91, 1.92, 1.914, 0.0003);

  EXPECT_NEAR(result.speed, 1.9103, 1e-12);
  EXPECT_TRUE(result.active);
  EXPECT_TRUE(result.command_valid);
  EXPECT_FALSE(result.target_not_above_ramp);
}

TEST(LaunchRamp, HandoffReachesTargetAndReleases) {
  const auto result =
      launch_ramp::advance_handoff(1.91, 1.92, 1.914, 0.02);

  EXPECT_DOUBLE_EQ(result.speed, 1.92);
  EXPECT_FALSE(result.active);
  EXPECT_TRUE(result.command_valid);
  EXPECT_FALSE(result.target_not_above_ramp);
}

TEST(LaunchRamp, HandoffProgressesWhenNormalBlendIsDisabled) {
  const auto result =
      launch_ramp::advance_handoff(1.77, 1.92, 1.77, 0.03);

  EXPECT_NEAR(result.speed, 1.80, 1e-12);
  EXPECT_TRUE(result.active);
  EXPECT_TRUE(result.command_valid);
  EXPECT_FALSE(result.target_not_above_ramp);
}

TEST(LaunchRamp, HandoffReturnsLowerTargetToNormalDeceleration) {
  const auto result =
      launch_ramp::advance_handoff(2.0, 1.2, 1.68, 0.03);

  EXPECT_DOUBLE_EQ(result.speed, 2.0);
  EXPECT_FALSE(result.active);
  EXPECT_FALSE(result.command_valid);
  EXPECT_TRUE(result.target_not_above_ramp);
}
