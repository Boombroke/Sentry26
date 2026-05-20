#include "sentry_behavior/plugins/condition/is_ally_base_hp_low.hpp"

namespace sentry_behavior
{

IsAllyBaseHpLowCondition::IsAllyBaseHpLowCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAllyBaseHpLowCondition::checkBaseHp, this), config)
{
}

BT::NodeStatus IsAllyBaseHpLowCondition::checkBaseHp()
{
  int threshold;
  auto msg = getInput<rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameRobotHP message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("threshold", threshold);

  return (msg->ally_base_hp < static_cast<uint16_t>(threshold)) ? BT::NodeStatus::SUCCESS
                                                                : BT::NodeStatus::FAILURE;
}

BT::PortsList IsAllyBaseHpLowCondition::providedPorts()
{
  return {
    BT::InputPort<rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "GameRobotHP port on blackboard"),
    BT::InputPort<int>("threshold", 5000, "Base HP below this value returns SUCCESS")
  };
}
}  // namespace sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_behavior::IsAllyBaseHpLowCondition>("IsAllyBaseHpLow");
}
