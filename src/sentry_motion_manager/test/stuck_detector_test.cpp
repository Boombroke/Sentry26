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

#include "sentry_motion_manager/stuck_detector.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace sentry_motion_manager
{
namespace
{

StuckDetectorConfig defaultTestConfig()
{
  StuckDetectorConfig config;
  config.window_s = 2.0;
  config.cmd_threshold_mps = 0.05;
  config.position_threshold_m = 0.10;
  config.hold_s = 0.5;
  config.cooldown_s = 1.0;
  return config;
}

TEST(StuckDetectorTest, NoDataNoTrigger)
{
  StuckDetector detector(defaultTestConfig());

  for (double t = 0.0; t < 5.0; t += 0.1) {
    EXPECT_FALSE(detector.tick(t));
  }
  EXPECT_FALSE(detector.triggerActive());
}

TEST(StuckDetectorTest, MovingForwardDoesNotTrigger)
{
  StuckDetector detector(defaultTestConfig());

  const double dt = 0.05;
  const double speed = 0.5;  // m/s
  for (int i = 0; i < 100; ++i) {
    const double t = i * dt;
    detector.addOdometry(t, speed * t, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    EXPECT_FALSE(detector.tick(t));
  }
  EXPECT_FALSE(detector.triggerActive());
}

TEST(StuckDetectorTest, CommandsButNoMovementTriggersAfterHold)
{
  StuckDetector detector(defaultTestConfig());

  const double dt = 0.05;
  bool triggered = false;
  double trigger_time = 0.0;
  // Feed for a duration > window_s + hold_s = 2.5s
  for (int i = 0; i < 80; ++i) {
    const double t = i * dt;
    detector.addOdometry(t, 0.0, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    if (detector.tick(t)) {
      triggered = true;
      trigger_time = t;
      break;
    }
  }
  ASSERT_TRUE(triggered);
  EXPECT_GE(trigger_time, 0.5);  // at least hold_s
  EXPECT_TRUE(detector.triggerActive());
}

TEST(StuckDetectorTest, HoldResetsOnProgress)
{
  StuckDetector detector(defaultTestConfig());

  const double dt = 0.05;
  // Phase 1: stuck for 0.5s-ish (less than hold_s won't trigger since window still warming up,
  // but ensures hold begins).
  int i = 0;
  for (; i < 10; ++i) {  // 0..0.45s
    const double t = i * dt;
    detector.addOdometry(t, 0.0, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    ASSERT_FALSE(detector.tick(t));
  }

  // Phase 2: start moving forward, displacement grows fast.
  const double speed = 1.0;
  double x = 0.0;
  for (; i < 80; ++i) {
    const double t = i * dt;
    x += speed * dt;
    detector.addOdometry(t, x, 0.0);
    detector.addCommandLinearSpeed(t, 0.5);
    EXPECT_FALSE(detector.tick(t));
  }
  EXPECT_FALSE(detector.triggerActive());
}

TEST(StuckDetectorTest, TriggerIsLatchedUntilNotified)
{
  StuckDetector detector(defaultTestConfig());

  const double dt = 0.05;
  bool triggered = false;
  double t = 0.0;
  for (int i = 0; i < 80 && !triggered; ++i) {
    t = i * dt;
    detector.addOdometry(t, 0.0, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    triggered = detector.tick(t);
  }
  ASSERT_TRUE(triggered);

  // Keep ticking without changing anything — should remain true.
  for (int j = 0; j < 5; ++j) {
    t += dt;
    detector.addOdometry(t, 0.0, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    EXPECT_TRUE(detector.tick(t));
  }

  // Notify recovery finished — trigger should clear immediately.
  detector.notifyRecoveryFinished(t);
  EXPECT_FALSE(detector.triggerActive());

  t += dt;
  detector.addOdometry(t, 0.0, 0.0);
  detector.addCommandLinearSpeed(t, 0.3);
  EXPECT_FALSE(detector.tick(t));
}

TEST(StuckDetectorTest, CooldownPreventsImmediateRetrigger)
{
  StuckDetectorConfig config = defaultTestConfig();
  config.cooldown_s = 1.0;
  StuckDetector detector(config);

  const double dt = 0.05;
  bool triggered = false;
  double t = 0.0;
  for (int i = 0; i < 80 && !triggered; ++i) {
    t = i * dt;
    detector.addOdometry(t, 0.0, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    triggered = detector.tick(t);
  }
  ASSERT_TRUE(triggered);
  const double trigger_end = t;
  detector.notifyRecoveryFinished(trigger_end);

  // Within cooldown window: should not retrigger even though conditions are met.
  bool retriggered = false;
  int steps_within_cooldown = 0;
  for (int i = 0; i < 18; ++i) {  // 18 * 0.05 = 0.9s < cooldown_s
    t += dt;
    detector.addOdometry(t, 0.0, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    if (detector.tick(t)) {
      retriggered = true;
      break;
    }
    ++steps_within_cooldown;
  }
  EXPECT_FALSE(retriggered);
  EXPECT_GT(steps_within_cooldown, 0);

  // After cooldown + additional hold_s, expect retrigger.
  bool retriggered_after = false;
  for (int i = 0; i < 80; ++i) {
    t += dt;
    detector.addOdometry(t, 0.0, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    if (detector.tick(t)) {
      retriggered_after = true;
      break;
    }
  }
  EXPECT_TRUE(retriggered_after);
  EXPECT_GE(t - trigger_end, config.cooldown_s);
}

TEST(StuckDetectorTest, OscillationNearOriginStillStuck)
{
  StuckDetector detector(defaultTestConfig());

  const double dt = 0.05;
  bool triggered = false;
  double t = 0.0;
  for (int i = 0; i < 80 && !triggered; ++i) {
    t = i * dt;
    // Oscillate within ±1cm — well below 10cm threshold.
    const double x = (i % 2 == 0) ? 0.01 : -0.01;
    detector.addOdometry(t, x, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    triggered = detector.tick(t);
  }
  EXPECT_TRUE(triggered);
  EXPECT_TRUE(detector.triggerActive());
  // Sanity: displacement metric should reflect the spread (~0.02m), still < threshold.
  EXPECT_LT(detector.lastWindowDisplacement(), 0.10);
}

TEST(StuckDetectorTest, ConfigureSanitizesNonPositiveValues)
{
  StuckDetectorConfig bad_config;
  bad_config.window_s = -1.0;
  bad_config.cmd_threshold_mps = 0.0;
  bad_config.position_threshold_m = -0.5;
  bad_config.hold_s = std::nan("");
  bad_config.cooldown_s = -3.0;

  StuckDetector detector(bad_config);
  // If sanitized, same inputs-as-default test should work like default config.
  const double dt = 0.05;
  for (int i = 0; i < 20; ++i) {
    const double t = i * dt;
    detector.addOdometry(t, 0.0, 0.0);
    detector.addCommandLinearSpeed(t, 0.3);
    detector.tick(t);
  }
  // With defaults (window=3s, hold=1s), at t=0.95s we should not have triggered yet.
  EXPECT_FALSE(detector.triggerActive());
}

}  // namespace
}  // namespace sentry_motion_manager
