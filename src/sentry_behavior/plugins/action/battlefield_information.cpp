#include "sentry_behavior/plugins/action/battlefield_information.hpp"

namespace sentry_behavior
{
BattlefieldInformationAction::BattlefieldInformationAction(
const std::string& name,const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config),
logger_(rclcpp::get_logger("BattlefieldInformationAction"))
{}

BT::PortsList BattlefieldInformationAction::providedPorts(){

    return{
      BT::InputPort<rm_interfaces::msg::GameRobotHP>(
        "key_port", "{@referee_allRobotHP}", "allRobotHP port on blackboard"),
        BT::OutputPort<std::string>("weight")
    };

}

BT::NodeStatus BattlefieldInformationAction::tick() {
  auto msg = getInput<rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "allRobotHP message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int ally_total = msg->ally_1_robot_hp + msg->ally_3_robot_hp +
                   msg->ally_4_robot_hp + msg->ally_7_robot_hp;

  if (ally_total > 1200) {
    setOutput("weight", "2.0");
  } else if (ally_total > 400) {
    setOutput("weight", "1.0");
  } else {
    setOutput("weight", "0.0");
  }

  return BT::NodeStatus::SUCCESS;
}


}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_behavior::BattlefieldInformationAction>("BattlefieldInformation");
}
