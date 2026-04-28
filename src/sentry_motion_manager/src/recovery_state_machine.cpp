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

#include <algorithm>
#include <cmath>

namespace sentry_motion_manager
{

namespace
{

constexpr double kMinimumPositiveValue = 1.0e-3;

double yawFromOdometry(const nav_msgs::msg::Odometry & odometry)
{
  const auto & q = odometry.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double sanitizePositive(double value, double fallback)
{
  if (std::isfinite(value) && value > 0.0) {
    return value;
  }
  return fallback;
}

}  // namespace

RecoveryStateMachine::RecoveryStateMachine(const RecoveryStateMachineConfig & config)
{
  configure(config);
}

void RecoveryStateMachine::configure(const RecoveryStateMachineConfig & config)
{
  config_ = config;
  config_.diagnose_timeout_s = sanitizePositive(config_.diagnose_timeout_s, 1.0);
  config_.no_progress_timeout_s = sanitizePositive(config_.no_progress_timeout_s, 1.0);
  config_.min_progress_delta_m = std::max(config_.min_progress_delta_m, kMinimumPositiveValue);
  config_.minimum_projected_success_m =
    std::max(config_.minimum_projected_success_m, kMinimumPositiveValue);

  config_.straight_release_distance_m = sanitizePositive(config_.straight_release_distance_m, 0.25);
  config_.straight_release_speed_mps = -std::abs(config_.straight_release_speed_mps);
  config_.straight_release_timeout_s = sanitizePositive(config_.straight_release_timeout_s, 3.0);

  config_.low_curvature_release_distance_m =
    sanitizePositive(config_.low_curvature_release_distance_m, 0.20);
  config_.low_curvature_release_speed_mps = -std::abs(config_.low_curvature_release_speed_mps);
  config_.low_curvature_angular_z_radps = std::abs(config_.low_curvature_angular_z_radps);
  config_.low_curvature_max_angular_z_radps =
    sanitizePositive(config_.low_curvature_max_angular_z_radps, 0.50);
  config_.low_curvature_release_timeout_s =
    sanitizePositive(config_.low_curvature_release_timeout_s, 3.0);

  config_.arc_escape_distance_m = sanitizePositive(config_.arc_escape_distance_m, 0.35);
  config_.arc_escape_speed_mps = -std::abs(config_.arc_escape_speed_mps);
  config_.arc_escape_angular_z_radps = std::abs(config_.arc_escape_angular_z_radps);
  config_.arc_escape_timeout_s = sanitizePositive(config_.arc_escape_timeout_s, 3.5);
}

void RecoveryStateMachine::start(double now_s)
{
  recovery_requested_ = true;
  phase_ = RecoveryPhase::kIdleDiagnose;
  request_time_s_ = now_s;
  phase_start_time_s_ = now_s;
  last_progress_time_s_ = now_s;
  best_projected_progress_m_ = 0.0;
  progress_checkpoint_m_ = 0.0;
  current_projected_progress_m_ = 0.0;
}

void RecoveryStateMachine::cancel()
{
  recovery_requested_ = false;
  phase_ = RecoveryPhase::kIdleDiagnose;
  best_projected_progress_m_ = 0.0;
  progress_checkpoint_m_ = 0.0;
  current_projected_progress_m_ = 0.0;
}

void RecoveryStateMachine::update(double now_s)
{
  if (!recovery_requested_) {
    return;
  }

  if (phase_ == RecoveryPhase::kIdleDiagnose) {
    if (has_odom_) {
      beginCommandPhase(RecoveryPhase::kStraightRelease, now_s);
      return;
    }
    if (now_s - request_time_s_ >= config_.diagnose_timeout_s) {
      finish(false);
    }
    return;
  }

  if (phase_ == RecoveryPhase::kSucceeded || phase_ == RecoveryPhase::kFailed) {
    return;
  }

  updateProgress(now_s);
  const auto limits = limitsForPhase(phase_);
  if (best_projected_progress_m_ >= limits.target_progress_m) {
    finish(true);
    return;
  }

  const bool phase_timed_out = now_s - phase_start_time_s_ >= limits.timeout_s;
  const bool progress_timed_out = now_s - last_progress_time_s_ >= config_.no_progress_timeout_s;
  if (phase_timed_out || progress_timed_out) {
    failCurrentPhase(now_s);
  }
}

void RecoveryStateMachine::updateOdometry(const nav_msgs::msg::Odometry & odometry)
{
  latest_pose_.x = odometry.pose.pose.position.x;
  latest_pose_.y = odometry.pose.pose.position.y;
  latest_pose_.yaw = yawFromOdometry(odometry);
  has_odom_ = true;
}

bool RecoveryStateMachine::isRunning() const
{
  return recovery_requested_ && phase_ != RecoveryPhase::kSucceeded &&
         phase_ != RecoveryPhase::kFailed;
}

bool RecoveryStateMachine::hasCommand() const
{
  return phase_ == RecoveryPhase::kStraightRelease ||
         phase_ == RecoveryPhase::kLowCurvatureRelease || phase_ == RecoveryPhase::kArcEscape;
}

RecoveryPhase RecoveryStateMachine::phase() const { return phase_; }

double RecoveryStateMachine::projectedProgress() const { return best_projected_progress_m_; }

geometry_msgs::msg::TwistStamped RecoveryStateMachine::command(
  const builtin_interfaces::msg::Time & stamp) const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = stamp;
  command.header.frame_id = config_.command_frame_id;

  if (!hasCommand()) {
    return command;
  }

  command.twist.linear.x = reverseSpeedForPhase(phase_);
  command.twist.angular.z = angularSpeedForPhase(phase_);
  return command;
}

void RecoveryStateMachine::beginCommandPhase(RecoveryPhase phase, double now_s)
{
  phase_ = phase;
  phase_start_pose_ = latest_pose_;
  escape_direction_x_ = -std::cos(latest_pose_.yaw);
  escape_direction_y_ = -std::sin(latest_pose_.yaw);
  phase_start_time_s_ = now_s;
  last_progress_time_s_ = now_s;
  best_projected_progress_m_ = 0.0;
  progress_checkpoint_m_ = 0.0;
  current_projected_progress_m_ = 0.0;
}

void RecoveryStateMachine::failCurrentPhase(double now_s)
{
  switch (phase_) {
    case RecoveryPhase::kStraightRelease:
      beginCommandPhase(RecoveryPhase::kLowCurvatureRelease, now_s);
      return;
    case RecoveryPhase::kLowCurvatureRelease:
      beginCommandPhase(RecoveryPhase::kArcEscape, now_s);
      return;
    case RecoveryPhase::kArcEscape:
      finish(false);
      return;
    case RecoveryPhase::kIdleDiagnose:
    case RecoveryPhase::kSucceeded:
    case RecoveryPhase::kFailed:
      finish(false);
      return;
  }
}

void RecoveryStateMachine::updateProgress(double now_s)
{
  current_projected_progress_m_ = (latest_pose_.x - phase_start_pose_.x) * escape_direction_x_ +
                                  (latest_pose_.y - phase_start_pose_.y) * escape_direction_y_;
  if (current_projected_progress_m_ > best_projected_progress_m_) {
    best_projected_progress_m_ = current_projected_progress_m_;
  }

  if (best_projected_progress_m_ - progress_checkpoint_m_ >= config_.min_progress_delta_m) {
    progress_checkpoint_m_ = best_projected_progress_m_;
    last_progress_time_s_ = now_s;
  }
}

void RecoveryStateMachine::finish(bool succeeded)
{
  recovery_requested_ = false;
  phase_ = succeeded ? RecoveryPhase::kSucceeded : RecoveryPhase::kFailed;
}

RecoveryStateMachine::PhaseLimits RecoveryStateMachine::limitsForPhase(RecoveryPhase phase) const
{
  PhaseLimits limits;
  switch (phase) {
    case RecoveryPhase::kStraightRelease:
      limits.target_progress_m = config_.straight_release_distance_m;
      limits.timeout_s = config_.straight_release_timeout_s;
      break;
    case RecoveryPhase::kLowCurvatureRelease:
      limits.target_progress_m = config_.low_curvature_release_distance_m;
      limits.timeout_s = config_.low_curvature_release_timeout_s;
      break;
    case RecoveryPhase::kArcEscape:
      limits.target_progress_m = config_.arc_escape_distance_m;
      limits.timeout_s = config_.arc_escape_timeout_s;
      break;
    case RecoveryPhase::kIdleDiagnose:
    case RecoveryPhase::kSucceeded:
    case RecoveryPhase::kFailed:
      limits.target_progress_m = config_.minimum_projected_success_m;
      limits.timeout_s = config_.diagnose_timeout_s;
      break;
  }

  limits.target_progress_m =
    std::max(limits.target_progress_m, config_.minimum_projected_success_m);
  return limits;
}

double RecoveryStateMachine::reverseSpeedForPhase(RecoveryPhase phase) const
{
  switch (phase) {
    case RecoveryPhase::kStraightRelease:
      return config_.straight_release_speed_mps;
    case RecoveryPhase::kLowCurvatureRelease:
      return config_.low_curvature_release_speed_mps;
    case RecoveryPhase::kArcEscape:
      return config_.arc_escape_speed_mps;
    case RecoveryPhase::kIdleDiagnose:
    case RecoveryPhase::kSucceeded:
    case RecoveryPhase::kFailed:
      return 0.0;
  }

  return 0.0;
}

double RecoveryStateMachine::angularSpeedForPhase(RecoveryPhase phase) const
{
  switch (phase) {
    case RecoveryPhase::kStraightRelease:
      return 0.0;
    case RecoveryPhase::kLowCurvatureRelease:
      return normalizedTurnDirection() *
             std::min(
               config_.low_curvature_angular_z_radps, config_.low_curvature_max_angular_z_radps);
    case RecoveryPhase::kArcEscape:
      return normalizedTurnDirection() * config_.arc_escape_angular_z_radps;
    case RecoveryPhase::kIdleDiagnose:
    case RecoveryPhase::kSucceeded:
    case RecoveryPhase::kFailed:
      return 0.0;
  }

  return 0.0;
}

double RecoveryStateMachine::normalizedTurnDirection() const
{
  return config_.turn_direction < 0.0 ? -1.0 : 1.0;
}

}  // namespace sentry_motion_manager
