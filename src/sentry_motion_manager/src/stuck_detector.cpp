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

#include <algorithm>
#include <cmath>

namespace sentry_motion_manager
{

namespace
{

double sanitizePositive(double value, double fallback)
{
  if (std::isfinite(value) && value > 0.0) {
    return value;
  }
  return fallback;
}

}  // namespace

StuckDetector::StuckDetector(const StuckDetectorConfig & config) { configure(config); }

void StuckDetector::configure(const StuckDetectorConfig & config)
{
  const StuckDetectorConfig defaults;
  config_ = config;
  config_.window_s = sanitizePositive(config_.window_s, defaults.window_s);
  config_.cmd_threshold_mps =
    sanitizePositive(config_.cmd_threshold_mps, defaults.cmd_threshold_mps);
  config_.position_threshold_m =
    sanitizePositive(config_.position_threshold_m, defaults.position_threshold_m);
  config_.hold_s = sanitizePositive(config_.hold_s, defaults.hold_s);
  config_.cooldown_s = sanitizePositive(config_.cooldown_s, defaults.cooldown_s);
}

void StuckDetector::addOdometry(double time_s, double x, double y)
{
  if (!std::isfinite(time_s) || !std::isfinite(x) || !std::isfinite(y)) {
    return;
  }
  OdomSample sample;
  sample.time_s = time_s;
  sample.x = x;
  sample.y = y;
  odom_samples_.push_back(sample);
  pruneOdometry(time_s - config_.window_s);
}

void StuckDetector::addCommandLinearSpeed(double time_s, double abs_linear_x)
{
  if (!std::isfinite(time_s) || !std::isfinite(abs_linear_x)) {
    return;
  }
  CommandSample sample;
  sample.time_s = time_s;
  sample.abs_linear_x = std::abs(abs_linear_x);
  cmd_samples_.push_back(sample);
  pruneCommands(time_s - config_.window_s);
}

bool StuckDetector::tick(double now_s)
{
  const double cutoff = now_s - config_.window_s;
  pruneOdometry(cutoff);
  pruneCommands(cutoff);

  // 样本不足时不触发且不更新 hold
  if (odom_samples_.size() < 2 || cmd_samples_.size() < 3) {
    hold_active_ = false;
    last_cmd_mean_ = computeCommandMean();
    last_displacement_ = computeMaxDisplacement();
    return trigger_active_;
  }

  last_cmd_mean_ = computeCommandMean();
  last_displacement_ = computeMaxDisplacement();

  const bool condition_a = last_cmd_mean_ > config_.cmd_threshold_mps;
  const bool condition_b = last_displacement_ < config_.position_threshold_m;

  if (condition_a && condition_b) {
    if (!hold_active_) {
      hold_active_ = true;
      hold_start_s_ = now_s;
    }
  } else {
    hold_active_ = false;
  }

  if (trigger_active_) {
    return true;
  }

  if (!hold_active_) {
    return false;
  }

  const double hold_elapsed = now_s - hold_start_s_;
  const double cooldown_elapsed = now_s - last_trigger_end_s_;
  if (hold_elapsed >= config_.hold_s && cooldown_elapsed >= config_.cooldown_s) {
    trigger_active_ = true;
    return true;
  }

  return false;
}

void StuckDetector::notifyRecoveryFinished(double now_s)
{
  trigger_active_ = false;
  hold_active_ = false;
  last_trigger_end_s_ = now_s;
}

bool StuckDetector::triggerActive() const { return trigger_active_; }

double StuckDetector::lastWindowCmdMean() const { return last_cmd_mean_; }

double StuckDetector::lastWindowDisplacement() const { return last_displacement_; }

double StuckDetector::holdElapsed(double now_s) const
{
  if (!hold_active_) {
    return -1.0;
  }
  return now_s - hold_start_s_;
}

void StuckDetector::pruneOdometry(double cutoff_s)
{
  while (!odom_samples_.empty() && odom_samples_.front().time_s < cutoff_s) {
    odom_samples_.pop_front();
  }
}

void StuckDetector::pruneCommands(double cutoff_s)
{
  while (!cmd_samples_.empty() && cmd_samples_.front().time_s < cutoff_s) {
    cmd_samples_.pop_front();
  }
}

double StuckDetector::computeCommandMean() const
{
  if (cmd_samples_.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto & sample : cmd_samples_) {
    sum += sample.abs_linear_x;
  }
  return sum / static_cast<double>(cmd_samples_.size());
}

double StuckDetector::computeMaxDisplacement() const
{
  if (odom_samples_.size() < 2) {
    return 0.0;
  }
  const auto & anchor = odom_samples_.front();
  double max_dist = 0.0;
  for (const auto & sample : odom_samples_) {
    const double dx = sample.x - anchor.x;
    const double dy = sample.y - anchor.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > max_dist) {
      max_dist = dist;
    }
  }
  return max_dist;
}

}  // namespace sentry_motion_manager
