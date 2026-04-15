#include "sentry_behavior/plugins/condition/is_hp_add.hpp"

namespace sentry_behavior
{

IsHPAddCondition::IsHPAddCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHPAddCondition::checkRobotStatus, this), config)
{
}

BT::NodeStatus IsHPAddCondition::checkRobotStatus()
{
  
  auto msg = getInput<rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }
  bool add_ok=msg->add_ok;
  
  RCLCPP_INFO(logger_,"add_ok:%d",add_ok );

  return (add_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsHPAddCondition::providedPorts()
{
  return {
    BT::InputPort<rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    };
}
}  // namespace sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_behavior::IsHPAddCondition>("IsHPAdd");
}