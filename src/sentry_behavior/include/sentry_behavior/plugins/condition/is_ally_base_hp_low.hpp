#ifndef SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ALLY_BASE_HP_LOW_HPP_
#define SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ALLY_BASE_HP_LOW_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"

namespace sentry_behavior
{

class IsAllyBaseHpLowCondition : public BT::SimpleConditionNode
{
public:
  IsAllyBaseHpLowCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkBaseHp();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsAllyBaseHpLowCondition");
};
}  // namespace sentry_behavior

#endif
