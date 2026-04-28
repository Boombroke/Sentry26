// Copyright 2026 Boombroke
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sentry_motion_manager/recovery_state_machine.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace sentry_motion_manager
{
namespace
{

nav_msgs::msg::Odometry makeOdometry(double x, double y, double yaw)
{
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation.w = std::cos(yaw * 0.5);
  odometry.pose.pose.orientation.z = std::sin(yaw * 0.5);
  return odometry;
}

builtin_interfaces::msg::Time makeStamp(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(stamp.sec)) * 1.0e9);
  return stamp;
}

RecoveryStateMachineConfig fastTimeoutConfig()
{
  RecoveryStateMachineConfig config;
  config.diagnose_timeout_s = 0.2;
  config.no_progress_timeout_s = 0.5;
  config.straight_release_timeout_s = 5.0;
  config.low_curvature_release_timeout_s = 5.0;
  config.arc_escape_timeout_s = 5.0;
  config.straight_release_distance_m = 0.25;
  config.low_curvature_release_distance_m = 0.20;
  config.arc_escape_distance_m = 0.35;
  return config;
}

TEST(RecoveryStateMachineTest, FirstCommandPhaseIsPureReverse)
{
  RecoveryStateMachine state_machine(fastTimeoutConfig());

  state_machine.start(0.0);
  state_machine.update(0.0);
  EXPECT_EQ(RecoveryPhase::kIdleDiagnose, state_machine.phase());
  EXPECT_FALSE(state_machine.hasCommand());

  state_machine.updateOdometry(makeOdometry(0.0, 0.0, 0.0));
  state_machine.update(0.1);

  ASSERT_EQ(RecoveryPhase::kStraightRelease, state_machine.phase());
  ASSERT_TRUE(state_machine.hasCommand());
  const auto command = state_machine.command(makeStamp(0.1));
  EXPECT_LT(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(0.0, command.twist.angular.z);
}

TEST(RecoveryStateMachineTest, ProjectedProgressIgnoresAccumulatedJitter)
{
  RecoveryStateMachine state_machine(fastTimeoutConfig());
  state_machine.updateOdometry(makeOdometry(0.0, 0.0, 0.0));
  state_machine.start(0.0);
  state_machine.update(0.0);

  for (int i = 0; i < 6; ++i) {
    state_machine.updateOdometry(makeOdometry(-0.03, 0.0, 0.0));
    state_machine.update(0.05 + i * 0.05);
    state_machine.updateOdometry(makeOdometry(0.0, 0.0, 0.0));
    state_machine.update(0.075 + i * 0.05);
  }

  EXPECT_LT(state_machine.projectedProgress(), 0.05);
  EXPECT_EQ(RecoveryPhase::kStraightRelease, state_machine.phase());
}

TEST(RecoveryStateMachineTest, NoProgressTimeoutFailsPhasesInsteadOfSucceeding)
{
  RecoveryStateMachineConfig config = fastTimeoutConfig();
  config.low_curvature_angular_z_radps = 0.35;
  config.low_curvature_max_angular_z_radps = 0.50;
  config.arc_escape_angular_z_radps = 0.80;
  RecoveryStateMachine state_machine(config);

  state_machine.updateOdometry(makeOdometry(0.0, 0.0, 0.0));
  state_machine.start(0.0);
  state_machine.update(0.0);
  ASSERT_EQ(RecoveryPhase::kStraightRelease, state_machine.phase());

  state_machine.update(0.6);
  ASSERT_EQ(RecoveryPhase::kLowCurvatureRelease, state_machine.phase());
  auto command = state_machine.command(makeStamp(0.6));
  EXPECT_LT(command.twist.linear.x, 0.0);
  EXPECT_GT(std::abs(command.twist.angular.z), 0.0);
  EXPECT_LE(std::abs(command.twist.angular.z), 0.50);

  state_machine.update(1.2);
  ASSERT_EQ(RecoveryPhase::kArcEscape, state_machine.phase());
  command = state_machine.command(makeStamp(1.2));
  EXPECT_LT(command.twist.linear.x, 0.0);
  EXPECT_GT(std::abs(command.twist.angular.z), 0.50);

  state_machine.update(1.8);
  EXPECT_EQ(RecoveryPhase::kFailed, state_machine.phase());
  EXPECT_FALSE(state_machine.hasCommand());
}

TEST(RecoveryStateMachineTest, SuccessRequiresPositiveProjectionAlongEscapeDirection)
{
  RecoveryStateMachineConfig config = fastTimeoutConfig();
  config.no_progress_timeout_s = 1.0;
  config.straight_release_distance_m = 0.10;
  RecoveryStateMachine state_machine(config);

  state_machine.updateOdometry(makeOdometry(0.0, 0.0, 0.0));
  state_machine.start(0.0);
  state_machine.update(0.0);

  state_machine.updateOdometry(makeOdometry(1.0, 0.0, 0.0));
  state_machine.update(0.2);

  EXPECT_EQ(RecoveryPhase::kStraightRelease, state_machine.phase());
  EXPECT_DOUBLE_EQ(0.0, state_machine.projectedProgress());
}

TEST(RecoveryStateMachineTest, StraightReleaseCanSucceedWithProjectedReverseMotion)
{
  RecoveryStateMachineConfig config = fastTimeoutConfig();
  config.straight_release_distance_m = 0.10;
  RecoveryStateMachine state_machine(config);

  state_machine.updateOdometry(makeOdometry(0.0, 0.0, 0.0));
  state_machine.start(0.0);
  state_machine.update(0.0);

  state_machine.updateOdometry(makeOdometry(-0.11, 0.0, 0.0));
  state_machine.update(0.2);

  EXPECT_EQ(RecoveryPhase::kSucceeded, state_machine.phase());
  EXPECT_FALSE(state_machine.hasCommand());
}

}  // namespace
}  // namespace sentry_motion_manager
