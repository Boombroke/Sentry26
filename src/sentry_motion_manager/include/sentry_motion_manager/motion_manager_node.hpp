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

#ifndef SENTRY_MOTION_MANAGER__MOTION_MANAGER_NODE_HPP_
#define SENTRY_MOTION_MANAGER__MOTION_MANAGER_NODE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_motion_manager/motion_types.hpp"
#include "sentry_motion_manager/recovery_state_machine.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace sentry_motion_manager
{

class MotionManagerNode : public rclcpp::Node
{
public:
  explicit MotionManagerNode(const rclcpp::NodeOptions & options);

private:
  struct SourceSlot
  {
    MotionSource source;
    std::string topic;
    double timeout_s;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription;
    MotionCommand latest_command;
  };

  static constexpr std::size_t kSourceCount = 5;
  // Arbitration priority after emergency stop: manual > recovery > terrain > evasion > navigation.
  inline static constexpr std::array<MotionSource, kSourceCount> kPriorityOrder{
    MotionSource::kManual, MotionSource::kRecovery, MotionSource::kTerrain, MotionSource::kEvasion,
    MotionSource::kNavigation};

  void commandCallback(MotionSource source, const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void recoveryTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void updateSelectedCommand();
  void selectCommand(const rclcpp::Time & now);
  void publishRecoveryCommand(const rclcpp::Time & stamp);
  void publishCommand();
  void publishState();
  void publishDiagnostics();

  bool isCommandFresh(
    const MotionCommand & command, double timeout_s, const rclcpp::Time & now) const;
  double declarePositiveDouble(const std::string & name, double default_value);
  std::chrono::nanoseconds periodFromFrequency(double frequency_hz) const;
  std::size_t sourceIndex(MotionSource source) const;
  MotionMode modeForSource(MotionSource source) const;
  geometry_msgs::msg::TwistStamped zeroCommand(const rclcpp::Time & stamp) const;
  geometry_msgs::msg::TwistStamped clampCommand(
    const geometry_msgs::msg::TwistStamped & command, const rclcpp::Time & stamp) const;
  geometry_msgs::msg::TwistStamped limitAcceleration(
    const geometry_msgs::msg::TwistStamped & command, const rclcpp::Time & stamp) const;
  double limitAxisAcceleration(
    double target, double previous, double max_accel, double elapsed_s) const;
  void publishAndRemember(const geometry_msgs::msg::TwistStamped & command);

  std::array<SourceSlot, kSourceCount> source_slots_;
  MotionState state_;
  geometry_msgs::msg::TwistStamped selected_command_;
  geometry_msgs::msg::TwistStamped last_published_command_;
  rclcpp::Time last_publish_stamp_;
  bool has_published_command_;
  RecoveryStateMachine recovery_state_machine_;

  bool command_output_enabled_;
  bool recovery_trigger_active_;
  bool recovery_command_was_active_;
  std::string command_frame_id_;
  double manager_frequency_hz_;
  double output_frequency_hz_;
  double state_frequency_hz_;
  double diagnostics_frequency_hz_;
  double max_linear_x_;
  double max_angular_z_;
  double max_linear_accel_;
  double max_angular_accel_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_trigger_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr recovery_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::TimerBase::SharedPtr manager_timer_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace sentry_motion_manager

#endif  // SENTRY_MOTION_MANAGER__MOTION_MANAGER_NODE_HPP_
