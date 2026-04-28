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
#include "rclcpp/rclcpp.hpp"
#include "sentry_motion_manager/motion_types.hpp"
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

  void commandCallback(MotionSource source, const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void updateSelectedCommand();
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

  std::array<SourceSlot, kSourceCount> source_slots_;
  MotionState state_;
  geometry_msgs::msg::TwistStamped selected_command_;

  bool command_output_enabled_;
  std::string command_frame_id_;
  double manager_frequency_hz_;
  double output_frequency_hz_;
  double state_frequency_hz_;
  double diagnostics_frequency_hz_;
  double max_linear_x_;
  double max_angular_z_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::TimerBase::SharedPtr manager_timer_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace sentry_motion_manager

#endif  // SENTRY_MOTION_MANAGER__MOTION_MANAGER_NODE_HPP_
