#ifndef SENTRY_BEHAVIOR__PLUGINS__ACTION__BATTLEFIELD_INFORMATION_HPP_
#define SENTRY_BEHAVIOR__PLUGINS__ACTION__BATTLEFIELD_INFORMATION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"

namespace sentry_behavior
{

class IsOutpostOkCondition : public BT::SimpleConditionNode
{
public:
  IsOutpostOkCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Tick function for game status ports
   */
  BT::NodeStatus checkOutpost();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsOutpostOkCondition");
};
}  // namespace sentry_behavior

#endif