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

#include "sentry_motion_manager/stuck_detector_node.hpp"

#include <algorithm>
#include <cmath>

#include "rclcpp_components/register_node_macro.hpp"

namespace sentry_motion_manager
{

namespace
{

constexpr double kMinFrequencyHz = 0.1;

// Parse the String published by MotionManagerNode::publishState. The string layout is
// "mode=... source=... recovery_phase=<phase> recovery_projected_progress_m=... ...".
// Returns empty string when the field is missing so the caller can ignore the update.
std::string extractRecoveryPhase(const std::string & state)
{
  const std::string key = "recovery_phase=";
  const auto start = state.find(key);
  if (start == std::string::npos) {
    return {};
  }
  const auto value_start = start + key.size();
  const auto value_end = state.find(' ', value_start);
  if (value_end == std::string::npos) {
    return state.substr(value_start);
  }
  return state.substr(value_start, value_end - value_start);
}

}  // namespace

StuckDetectorNode::StuckDetectorNode(const rclcpp::NodeOptions & options)
: Node("stuck_detector", options)
{
  StuckDetectorConfig config;
  config.window_s = declarePositiveDouble("window_s", config.window_s);
  config.cmd_threshold_mps = declarePositiveDouble("cmd_threshold_mps", config.cmd_threshold_mps);
  config.position_threshold_m =
    declarePositiveDouble("position_threshold_m", config.position_threshold_m);
  config.hold_s = declarePositiveDouble("hold_s", config.hold_s);
  config.cooldown_s = declarePositiveDouble("cooldown_s", config.cooldown_s);
  detector_.configure(config);

  tick_frequency_hz_ = declarePositiveDouble("tick_frequency_hz", tick_frequency_hz_);

  const auto odometry_topic = this->declare_parameter<std::string>("odometry_topic", "odometry");
  const auto command_topic = this->declare_parameter<std::string>("command_topic", "cmd_vel");
  const auto motion_state_topic =
    this->declare_parameter<std::string>("motion_state_topic", "motion_manager/state");
  const auto recovery_trigger_topic = this->declare_parameter<std::string>(
    "recovery_trigger_topic", "motion_manager/recovery_trigger");

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic, rclcpp::SensorDataQoS(),
    std::bind(&StuckDetectorNode::odometryCallback, this, std::placeholders::_1));
  command_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    command_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&StuckDetectorNode::commandCallback, this, std::placeholders::_1));
  motion_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    motion_state_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&StuckDetectorNode::motionStateCallback, this, std::placeholders::_1));

  recovery_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>(recovery_trigger_topic, 1);

  tick_timer_ = this->create_wall_timer(
    periodFromFrequency(tick_frequency_hz_), std::bind(&StuckDetectorNode::tick, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Stuck detector started: window=%.2fs cmd_th=%.3f pos_th=%.3f hold=%.2fs cooldown=%.2fs",
    config.window_s, config.cmd_threshold_mps, config.position_threshold_m, config.hold_s,
    config.cooldown_s);
}

void StuckDetectorNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto t = rclcpp::Time(msg->header.stamp, this->now().get_clock_type()).seconds();
  detector_.addOdometry(t, msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void StuckDetectorNode::commandCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const auto t = rclcpp::Time(msg->header.stamp, this->now().get_clock_type()).seconds();
  detector_.addCommandLinearSpeed(t, std::abs(msg->twist.linear.x));
}

void StuckDetectorNode::motionStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const auto phase = extractRecoveryPhase(msg->data);
  if (phase.empty()) {
    return;
  }
  if ((phase == "succeeded" || phase == "failed") && detector_.triggerActive()) {
    const auto now_s = this->now().seconds();
    detector_.notifyRecoveryFinished(now_s);
    publishTrigger(false);
    RCLCPP_INFO(
      this->get_logger(), "Recovery reported %s, releasing trigger and entering cooldown.",
      phase.c_str());
  }
}

void StuckDetectorNode::tick()
{
  const auto now_s = this->now().seconds();
  const bool should_trigger = detector_.tick(now_s);

  if (should_trigger && (!has_published_trigger_ || !last_published_trigger_)) {
    publishTrigger(true);
    RCLCPP_WARN(
      this->get_logger(),
      "Stuck detected (cmd_mean=%.3f, max_disp=%.3fm); raising recovery_trigger.",
      detector_.lastWindowCmdMean(), detector_.lastWindowDisplacement());
  }
}

void StuckDetectorNode::publishTrigger(bool value)
{
  std_msgs::msg::Bool msg;
  msg.data = value;
  recovery_trigger_pub_->publish(msg);
  last_published_trigger_ = value;
  has_published_trigger_ = true;
}

double StuckDetectorNode::declarePositiveDouble(const std::string & name, double default_value)
{
  const double value = this->declare_parameter<double>(name, default_value);
  if (value > 0.0) {
    return value;
  }
  RCLCPP_WARN(
    this->get_logger(), "Parameter '%s' must be positive. Falling back to %.3f.", name.c_str(),
    default_value);
  return default_value;
}

std::chrono::nanoseconds StuckDetectorNode::periodFromFrequency(double frequency_hz) const
{
  const double safe_frequency = std::max(frequency_hz, kMinFrequencyHz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / safe_frequency));
}

}  // namespace sentry_motion_manager

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_motion_manager::StuckDetectorNode)
