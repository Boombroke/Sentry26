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

#ifndef SENTRY_MOTION_MANAGER__STUCK_DETECTOR_NODE_HPP_
#define SENTRY_MOTION_MANAGER__STUCK_DETECTOR_NODE_HPP_

#include <chrono>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_motion_manager/stuck_detector.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace sentry_motion_manager
{

class StuckDetectorNode : public rclcpp::Node
{
public:
  explicit StuckDetectorNode(const rclcpp::NodeOptions & options);

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void commandCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void motionStateCallback(const std_msgs::msg::String::SharedPtr msg);
  void tick();

  void publishTrigger(bool value);
  double declarePositiveDouble(const std::string & name, double default_value);
  std::chrono::nanoseconds periodFromFrequency(double frequency_hz) const;

  StuckDetector detector_;
  double tick_frequency_hz_{20.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_trigger_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  bool last_published_trigger_{false};
  bool has_published_trigger_{false};
};

}  // namespace sentry_motion_manager

#endif  // SENTRY_MOTION_MANAGER__STUCK_DETECTOR_NODE_HPP_
