#ifndef SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HP_ADD_HPP_
#define SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HP_ADD_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_behavior
{
/**
 * @brief A BT::ConditionNode that get GameStatus from port and
 * returns SUCCESS when current game status and remain time is expected
 */
class IsHPAddCondition : public BT::SimpleConditionNode
{
public:
  IsHPAddCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  int hp_last;
  /**
   * @brief Tick function for game status ports
   */
  BT::NodeStatus checkRobotStatus();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsHPAddCondition");
};
}  // namespace sentry_behavior

#endif  // SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_STATUS_OK_HPP_