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

#include "sentry_behavior/sentry_behavior_server.hpp"

#include <filesystem>
#include <fstream>
#include <unordered_set>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rm_interfaces/msg/buff.hpp"
#include "rm_interfaces/msg/event_data.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/ground_robot_position.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
namespace sentry_behavior
{

template <typename T>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  auto sub = node()->create_subscription<T>(
    topic, qos,
    [this, bb_key](const typename T::SharedPtr msg) { globalBlackboard()->set(bb_key, *msg); });
  subscriptions_.push_back(sub);
}

SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)
{
  node()->declare_parameter("use_cout_logger", false);
  node()->get_parameter("use_cout_logger", use_cout_logger_);

  subscribe<rm_interfaces::msg::EventData>("referee/event_data", "referee_eventData");
  subscribe<rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", "referee_allRobotHP");
  subscribe<rm_interfaces::msg::GameStatus>("referee/game_status", "referee_gameStatus");
  subscribe<rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");
  subscribe<rm_interfaces::msg::RfidStatus>("referee/rfid_status", "referee_rfidStatus");
  subscribe<rm_interfaces::msg::RobotStatus>("referee/robot_status", "referee_robotStatus");
  subscribe<rm_interfaces::msg::Buff>("referee/buff", "referee_buff");

  auto detector_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Armors>("detector/armors", "detector_armors", detector_qos);
  auto tracker_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Target>("tracker/target", "tracker_target", tracker_qos);

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);
}

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;

  // Workaround: Groot2 Free limits monitoring to ≤20 nodes, but BT.CPP 4.9+ createTree()
  // copies ALL factory manifests. Prune to only types used by the running tree.
  // Safe: unordered_map::erase does not invalidate references to remaining elements.
  std::unordered_set<std::string> used_types;
  tree.applyVisitor([&used_types](const BT::TreeNode * node) {
    if (node) {
      used_types.insert(node->registrationName());
    }
  });

  auto it = tree.manifests.begin();
  while (it != tree.manifests.end()) {
    if (used_types.count(it->first) == 0) {
      it = tree.manifests.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  return std::nullopt;
}

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  logger_cout_.reset();
  std::string result = treeName() +
                       " tree completed with status=" + std::to_string(static_cast<int>(status)) +
                       " after " + std::to_string(tick_count_) + " ticks";
  return result;
}

}  // namespace sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<sentry_behavior::SentryBehaviorServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  try {
    std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());
    std::ofstream file(std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml");
    file << xml_models;
  } catch (const std::exception & e) {
    std::cerr << "Failed to export BT node models: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
}
