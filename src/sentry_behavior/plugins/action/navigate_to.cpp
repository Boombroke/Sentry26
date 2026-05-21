// Copyright 2025 Lihan Chen
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

#include "sentry_behavior/plugins/action/navigate_to.hpp"

#include <cmath>

namespace sentry_behavior
{

NavigateToAction::NavigateToAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

BT::PortsList NavigateToAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<float>("goal_pose_x"),
    BT::InputPort<float>("goal_pose_y"),
    BT::InputPort<float>("goal_pose_yaw", 0.0f, "目标航向 (rad), 默认 0"),
    BT::OutputPort<int>("error_code"),
  });
}

bool NavigateToAction::setGoal(Goal & goal)
{
  auto x = getInput<float>("goal_pose_x");
  auto y = getInput<float>("goal_pose_y");
  if (!x) {
    throw BT::RuntimeError("error reading port [goal_pose_x]: ", x.error());
  }
  if (!y) {
    throw BT::RuntimeError("error reading port [goal_pose_y]: ", y.error());
  }
  float yaw = 0.0f;
  getInput<float>("goal_pose_yaw", yaw);

  auto node = node_.lock();
  goal.pose.header.stamp = node ? node->now() : rclcpp::Time(0);
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = x.value();
  goal.pose.pose.position.y = y.value();
  goal.pose.pose.position.z = 0.0;
  goal.pose.pose.orientation.x = 0.0;
  goal.pose.pose.orientation.y = 0.0;
  goal.pose.pose.orientation.z = std::sin(yaw * 0.5f);
  goal.pose.pose.orientation.w = std::cos(yaw * 0.5f);
  return true;
}

BT::NodeStatus NavigateToAction::onResultReceived(const WrappedResult & wr)
{
  // rclcpp_action 的 result code 透传到 blackboard, 上层可决定 retry / fallback 策略
  setOutput<int>("error_code", static_cast<int>(wr.code));
  return wr.code == rclcpp_action::ResultCode::SUCCEEDED ? BT::NodeStatus::SUCCESS
                                                         : BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigateToAction::onFailure(BT::ActionNodeErrorCode error)
{
  if (auto node = node_.lock()) {
    RCLCPP_WARN(node->get_logger(), "NavigateTo failure: %s", BT::toStr(error));
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(sentry_behavior::NavigateToAction, "NavigateTo");
