#include "sentry_behavior/plugins/condition/is_enemy_fortress_occupied.hpp"

namespace sentry_behavior
{
namespace
{
// 与电控 SendPacket.event_data / GameRobotHP.event_data 一致
constexpr uint32_t kAllyFortressOccupiedByEnemy = 2;
}  // namespace

IsEnemyFortressOccupiedCondition::IsEnemyFortressOccupiedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsEnemyFortressOccupiedCondition::checkFortressOccupied, this), config)
{
}

BT::NodeStatus IsEnemyFortressOccupiedCondition::checkFortressOccupied()
{
  auto msg = getInput<rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameRobotHP message is not available");
    return BT::NodeStatus::FAILURE;
  }

  // RMUC 阶段三巡逻: 己方堡垒被敌方占领时与 IsAllyBaseHpLow 并列，触发防御三点
  return msg->event_data == kAllyFortressOccupiedByEnemy ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
}

BT::PortsList IsEnemyFortressOccupiedCondition::providedPorts()
{
  return {
    BT::InputPort<rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "GameRobotHP port on blackboard")
  };
}
}  // namespace sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_behavior::IsEnemyFortressOccupiedCondition>(
    "IsEnemyFortressOccupied");
}
