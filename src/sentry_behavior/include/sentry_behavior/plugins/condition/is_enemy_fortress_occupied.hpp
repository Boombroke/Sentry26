#ifndef SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ENEMY_FORTRESS_OCCUPIED_HPP_
#define SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ENEMY_FORTRESS_OCCUPIED_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"

namespace sentry_behavior
{

class IsEnemyFortressOccupiedCondition : public BT::SimpleConditionNode
{
public:
  IsEnemyFortressOccupiedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkFortressOccupied();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsEnemyFortressOccupiedCondition");
};
}  // namespace sentry_behavior

#endif
