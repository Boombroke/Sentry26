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

#include "sentry_motion_manager/motion_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace sentry_motion_manager
{

namespace
{

constexpr double kMinFrequencyHz = 0.1;

bool isZeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

double finiteOrZero(double value) { return std::isfinite(value) ? value : 0.0; }

}  // namespace

std::string toString(MotionSource source)
{
  switch (source) {
    case MotionSource::kNavigation:
      return "navigation";
    case MotionSource::kRecovery:
      return "recovery";
    case MotionSource::kTerrain:
      return "terrain";
    case MotionSource::kEvasion:
      return "evasion";
    case MotionSource::kManual:
      return "manual";
  }

  return "unknown";
}

std::string toString(MotionMode mode)
{
  switch (mode) {
    case MotionMode::kIdle:
      return "idle";
    case MotionMode::kNavigation:
      return "navigation";
    case MotionMode::kRecovery:
      return "recovery";
    case MotionMode::kTerrainTraverse:
      return "terrain_traverse";
    case MotionMode::kEvasiveSpin:
      return "evasive_spin";
    case MotionMode::kManual:
      return "manual";
    case MotionMode::kEmergencyStop:
      return "emergency_stop";
  }

  return "unknown";
}

std::string toString(RecoveryPhase phase)
{
  switch (phase) {
    case RecoveryPhase::kInactive:
      return "inactive";
    case RecoveryPhase::kStraightReverse:
      return "straight_reverse";
    case RecoveryPhase::kArcEscape:
      return "arc_escape";
    case RecoveryPhase::kFailed:
      return "failed";
    case RecoveryPhase::kSucceeded:
      return "succeeded";
  }

  return "unknown";
}

MotionManagerNode::MotionManagerNode(const rclcpp::NodeOptions & options)
: Node("motion_manager", options),
  source_slots_{{
    {MotionSource::kManual, "", 0.0, nullptr, MotionCommand{MotionSource::kManual}},
    {MotionSource::kRecovery, "", 0.0, nullptr, MotionCommand{MotionSource::kRecovery}},
    {MotionSource::kTerrain, "", 0.0, nullptr, MotionCommand{MotionSource::kTerrain}},
    {MotionSource::kEvasion, "", 0.0, nullptr, MotionCommand{MotionSource::kEvasion}},
    {MotionSource::kNavigation, "", 0.0, nullptr, MotionCommand{MotionSource::kNavigation}},
  }},
  state_(),
  selected_command_(),
  last_published_command_(),
  last_publish_stamp_(0, 0, RCL_SYSTEM_TIME),
  has_published_command_(false),
  command_output_enabled_(false),
  command_frame_id_("base_footprint"),
  manager_frequency_hz_(100.0),
  output_frequency_hz_(50.0),
  state_frequency_hz_(10.0),
  diagnostics_frequency_hz_(2.0),
  max_linear_x_(2.0),
  max_angular_z_(6.3),
  max_linear_accel_(3.0),
  max_angular_accel_(12.0)
{
  command_output_enabled_ = this->declare_parameter<bool>("command_output_enabled", false);
  command_frame_id_ = this->declare_parameter<std::string>("command_frame_id", command_frame_id_);

  manager_frequency_hz_ = declarePositiveDouble("manager_frequency_hz", manager_frequency_hz_);
  output_frequency_hz_ = declarePositiveDouble("output_frequency_hz", output_frequency_hz_);
  state_frequency_hz_ = declarePositiveDouble("state_frequency_hz", state_frequency_hz_);
  diagnostics_frequency_hz_ =
    declarePositiveDouble("diagnostics_frequency_hz", diagnostics_frequency_hz_);
  max_linear_x_ = declarePositiveDouble("max_linear_x", max_linear_x_);
  max_angular_z_ = declarePositiveDouble("max_angular_z", max_angular_z_);
  max_linear_accel_ = declarePositiveDouble("max_linear_accel", max_linear_accel_);
  max_angular_accel_ = declarePositiveDouble("max_angular_accel", max_angular_accel_);

  source_slots_[sourceIndex(MotionSource::kNavigation)].topic =
    this->declare_parameter<std::string>("nav_command_topic", "cmd_vel_nav");
  source_slots_[sourceIndex(MotionSource::kRecovery)].topic =
    this->declare_parameter<std::string>("recovery_command_topic", "cmd_vel_recovery");
  source_slots_[sourceIndex(MotionSource::kTerrain)].topic =
    this->declare_parameter<std::string>("terrain_command_topic", "cmd_vel_terrain");
  source_slots_[sourceIndex(MotionSource::kEvasion)].topic =
    this->declare_parameter<std::string>("evasion_command_topic", "cmd_vel_evasion");
  source_slots_[sourceIndex(MotionSource::kManual)].topic =
    this->declare_parameter<std::string>("manual_command_topic", "cmd_vel_manual");

  source_slots_[sourceIndex(MotionSource::kNavigation)].timeout_s =
    declarePositiveDouble("nav_timeout_s", 0.25);
  source_slots_[sourceIndex(MotionSource::kRecovery)].timeout_s =
    declarePositiveDouble("recovery_timeout_s", 0.20);
  source_slots_[sourceIndex(MotionSource::kTerrain)].timeout_s =
    declarePositiveDouble("terrain_timeout_s", 0.20);
  source_slots_[sourceIndex(MotionSource::kEvasion)].timeout_s =
    declarePositiveDouble("evasion_timeout_s", 0.20);
  source_slots_[sourceIndex(MotionSource::kManual)].timeout_s =
    declarePositiveDouble("manual_timeout_s", 0.20);

  const auto emergency_stop_topic =
    this->declare_parameter<std::string>("emergency_stop_topic", "emergency_stop");
  const auto output_command_topic =
    this->declare_parameter<std::string>("output_command_topic", "cmd_vel");
  const auto state_topic =
    this->declare_parameter<std::string>("state_topic", "motion_manager/state");
  const auto diagnostics_topic =
    this->declare_parameter<std::string>("diagnostics_topic", "diagnostics");

  for (auto & slot : source_slots_) {
    slot.subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      slot.topic, rclcpp::SystemDefaultsQoS(),
      [this, source = slot.source](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        commandCallback(source, msg);
      });
  }

  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    emergency_stop_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&MotionManagerNode::emergencyStopCallback, this, std::placeholders::_1));

  command_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_command_topic, 1);
  state_pub_ = this->create_publisher<std_msgs::msg::String>(state_topic, 1);
  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 1);

  selected_command_ = zeroCommand(this->now());
  last_published_command_ = selected_command_;
  state_.output_enabled = command_output_enabled_;

  manager_timer_ = this->create_wall_timer(
    periodFromFrequency(manager_frequency_hz_),
    std::bind(&MotionManagerNode::updateSelectedCommand, this));
  output_timer_ = this->create_wall_timer(
    periodFromFrequency(output_frequency_hz_), std::bind(&MotionManagerNode::publishCommand, this));
  state_timer_ = this->create_wall_timer(
    periodFromFrequency(state_frequency_hz_), std::bind(&MotionManagerNode::publishState, this));
  diagnostics_timer_ = this->create_wall_timer(
    periodFromFrequency(diagnostics_frequency_hz_),
    std::bind(&MotionManagerNode::publishDiagnostics, this));

  RCLCPP_INFO(
    this->get_logger(), "Motion manager started with output %s. Publishing to '%s' in frame '%s'.",
    command_output_enabled_ ? "enabled" : "disabled", output_command_topic.c_str(),
    command_frame_id_.c_str());
}

void MotionManagerNode::commandCallback(
  MotionSource source, const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  auto & slot = source_slots_[sourceIndex(source)];
  slot.latest_command.source = source;
  slot.latest_command.twist = *msg;
  if (isZeroStamp(slot.latest_command.twist.header.stamp)) {
    slot.latest_command.twist.header.stamp = this->now();
  }
  slot.latest_command.valid = true;
}

void MotionManagerNode::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  state_.emergency_stop = msg->data;
}

void MotionManagerNode::updateSelectedCommand() { selectCommand(this->now()); }

void MotionManagerNode::selectCommand(const rclcpp::Time & now)
{
  state_.output_enabled = command_output_enabled_;

  if (state_.emergency_stop) {
    state_.mode = MotionMode::kEmergencyStop;
    state_.has_fresh_command = false;
    selected_command_ = zeroCommand(now);
    return;
  }

  for (const auto source : kPriorityOrder) {
    const auto & slot = source_slots_[sourceIndex(source)];
    if (isCommandFresh(slot.latest_command, slot.timeout_s, now)) {
      state_.mode = modeForSource(slot.source);
      state_.selected_source = slot.source;
      state_.has_fresh_command = true;
      selected_command_ = clampCommand(slot.latest_command.twist, now);
      return;
    }
  }

  state_.mode = MotionMode::kIdle;
  state_.has_fresh_command = false;
  selected_command_ = zeroCommand(now);
}

void MotionManagerNode::publishCommand()
{
  const auto now = this->now();
  selectCommand(now);

  if (!command_output_enabled_ || state_.emergency_stop || !state_.has_fresh_command) {
    publishAndRemember(zeroCommand(now));
    return;
  }

  publishAndRemember(limitAcceleration(selected_command_, now));
}

void MotionManagerNode::publishState()
{
  std_msgs::msg::String msg;
  std::ostringstream stream;
  stream << "mode=" << toString(state_.mode) << " source=" << toString(state_.selected_source)
         << " recovery_phase=" << toString(state_.recovery_phase)
         << " output_enabled=" << (state_.output_enabled ? "true" : "false")
         << " emergency_stop=" << (state_.emergency_stop ? "true" : "false")
         << " has_fresh_command=" << (state_.has_fresh_command ? "true" : "false");
  msg.data = stream.str();
  state_pub_->publish(msg);
}

void MotionManagerNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "sentry_motion_manager";
  status.hardware_id = "motion_manager";
  status.level = state_.emergency_stop ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                       : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = state_.emergency_stop ? "emergency stop active" : "motion manager ready";
  status.values.push_back(makeKeyValue("mode", toString(state_.mode)));
  status.values.push_back(makeKeyValue("selected_source", toString(state_.selected_source)));
  status.values.push_back(makeKeyValue("output_enabled", state_.output_enabled ? "true" : "false"));
  status.values.push_back(
    makeKeyValue("has_fresh_command", state_.has_fresh_command ? "true" : "false"));

  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);
}

bool MotionManagerNode::isCommandFresh(
  const MotionCommand & command, double timeout_s, const rclcpp::Time & now) const
{
  if (!command.valid) {
    return false;
  }

  const rclcpp::Time command_stamp(command.twist.header.stamp, now.get_clock_type());
  const auto age = now - command_stamp;
  return age.seconds() >= 0.0 && age.seconds() <= timeout_s;
}

double MotionManagerNode::declarePositiveDouble(const std::string & name, double default_value)
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

std::chrono::nanoseconds MotionManagerNode::periodFromFrequency(double frequency_hz) const
{
  const double safe_frequency = std::max(frequency_hz, kMinFrequencyHz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / safe_frequency));
}

std::size_t MotionManagerNode::sourceIndex(MotionSource source) const
{
  switch (source) {
    case MotionSource::kManual:
      return 0;
    case MotionSource::kRecovery:
      return 1;
    case MotionSource::kTerrain:
      return 2;
    case MotionSource::kEvasion:
      return 3;
    case MotionSource::kNavigation:
      return 4;
  }

  return 4;
}

MotionMode MotionManagerNode::modeForSource(MotionSource source) const
{
  switch (source) {
    case MotionSource::kManual:
      return MotionMode::kManual;
    case MotionSource::kRecovery:
      return MotionMode::kRecovery;
    case MotionSource::kTerrain:
      return MotionMode::kTerrainTraverse;
    case MotionSource::kEvasion:
      return MotionMode::kEvasiveSpin;
    case MotionSource::kNavigation:
      return MotionMode::kNavigation;
  }

  return MotionMode::kIdle;
}

geometry_msgs::msg::TwistStamped MotionManagerNode::zeroCommand(const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = stamp;
  command.header.frame_id = command_frame_id_;
  return command;
}

geometry_msgs::msg::TwistStamped MotionManagerNode::clampCommand(
  const geometry_msgs::msg::TwistStamped & command, const rclcpp::Time & stamp) const
{
  auto clamped = command;
  clamped.header.stamp = stamp;
  if (clamped.header.frame_id.empty()) {
    clamped.header.frame_id = command_frame_id_;
  }

  clamped.twist.linear.x =
    std::clamp(finiteOrZero(clamped.twist.linear.x), -max_linear_x_, max_linear_x_);
  clamped.twist.linear.y = 0.0;
  clamped.twist.linear.z = 0.0;
  clamped.twist.angular.x = 0.0;
  clamped.twist.angular.y = 0.0;
  clamped.twist.angular.z =
    std::clamp(finiteOrZero(clamped.twist.angular.z), -max_angular_z_, max_angular_z_);
  return clamped;
}

geometry_msgs::msg::TwistStamped MotionManagerNode::limitAcceleration(
  const geometry_msgs::msg::TwistStamped & command, const rclcpp::Time & stamp) const
{
  auto limited = clampCommand(command, stamp);

  const double previous_linear_x =
    has_published_command_ ? last_published_command_.twist.linear.x : 0.0;
  const double previous_angular_z =
    has_published_command_ ? last_published_command_.twist.angular.z : 0.0;
  const double elapsed_s =
    has_published_command_ ? (stamp - last_publish_stamp_).seconds() : (1.0 / output_frequency_hz_);

  limited.twist.linear.x =
    limitAxisAcceleration(limited.twist.linear.x, previous_linear_x, max_linear_accel_, elapsed_s);
  limited.twist.angular.z = limitAxisAcceleration(
    limited.twist.angular.z, previous_angular_z, max_angular_accel_, elapsed_s);
  return limited;
}

double MotionManagerNode::limitAxisAcceleration(
  double target, double previous, double max_accel, double elapsed_s) const
{
  if (elapsed_s <= 0.0) {
    return previous;
  }

  const double max_delta = max_accel * elapsed_s;
  return std::clamp(target, previous - max_delta, previous + max_delta);
}

void MotionManagerNode::publishAndRemember(const geometry_msgs::msg::TwistStamped & command)
{
  command_pub_->publish(command);
  last_published_command_ = command;
  last_publish_stamp_ = rclcpp::Time(command.header.stamp, this->now().get_clock_type());
  has_published_command_ = true;
}

}  // namespace sentry_motion_manager

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_motion_manager::MotionManagerNode)
