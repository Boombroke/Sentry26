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

#ifndef SENTRY_MOTION_MANAGER__RECOVERY_STATE_MACHINE_HPP_
#define SENTRY_MOTION_MANAGER__RECOVERY_STATE_MACHINE_HPP_

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sentry_motion_manager/motion_types.hpp"

namespace sentry_motion_manager
{

struct RecoveryStateMachineConfig
{
  std::string command_frame_id{"base_footprint"};

  double diagnose_timeout_s{1.0};
  double no_progress_timeout_s{1.0};
  double min_progress_delta_m{0.02};
  double minimum_projected_success_m{0.05};
  double turn_direction{1.0};

  double straight_release_distance_m{0.25};
  double straight_release_speed_mps{-0.15};
  double straight_release_timeout_s{3.0};

  double low_curvature_release_distance_m{0.20};
  double low_curvature_release_speed_mps{-0.15};
  double low_curvature_angular_z_radps{0.35};
  double low_curvature_max_angular_z_radps{0.50};
  double low_curvature_release_timeout_s{3.0};

  double arc_escape_distance_m{0.35};
  double arc_escape_speed_mps{-0.20};
  double arc_escape_angular_z_radps{0.80};
  double arc_escape_timeout_s{3.5};
};

class RecoveryStateMachine
{
public:
  explicit RecoveryStateMachine(const RecoveryStateMachineConfig & config = {});

  void configure(const RecoveryStateMachineConfig & config);
  void start(double now_s);
  void cancel();
  void update(double now_s);
  void updateOdometry(const nav_msgs::msg::Odometry & odometry);

  bool isRunning() const;
  bool hasCommand() const;
  RecoveryPhase phase() const;
  double projectedProgress() const;
  geometry_msgs::msg::TwistStamped command(const builtin_interfaces::msg::Time & stamp) const;

private:
  struct PlanarPose
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct PhaseLimits
  {
    double target_progress_m{0.0};
    double timeout_s{0.0};
  };

  void beginCommandPhase(RecoveryPhase phase, double now_s);
  void failCurrentPhase(double now_s);
  void updateProgress(double now_s);
  void finish(bool succeeded);
  PhaseLimits limitsForPhase(RecoveryPhase phase) const;
  double reverseSpeedForPhase(RecoveryPhase phase) const;
  double angularSpeedForPhase(RecoveryPhase phase) const;
  double normalizedTurnDirection() const;

  RecoveryStateMachineConfig config_;
  RecoveryPhase phase_{RecoveryPhase::kIdleDiagnose};
  bool recovery_requested_{false};
  bool has_odom_{false};

  PlanarPose latest_pose_;
  PlanarPose phase_start_pose_;
  double escape_direction_x_{0.0};
  double escape_direction_y_{0.0};
  double request_time_s_{0.0};
  double phase_start_time_s_{0.0};
  double last_progress_time_s_{0.0};
  double best_projected_progress_m_{0.0};
  double progress_checkpoint_m_{0.0};
  double current_projected_progress_m_{0.0};
};

}  // namespace sentry_motion_manager

#endif  // SENTRY_MOTION_MANAGER__RECOVERY_STATE_MACHINE_HPP_
