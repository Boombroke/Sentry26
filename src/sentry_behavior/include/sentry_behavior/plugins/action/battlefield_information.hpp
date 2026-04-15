#ifndef SENTRY_BEHAVIOR__PLUGINS__ACTION__BATTLEFIELD_INFORMATION_HPP_
#define SENTRY_BEHAVIOR__PLUGINS__ACTION__BATTLEFIELD_INFORMATION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"

namespace sentry_behavior
{
    class BattlefieldInformationAction : public BT::SyncActionNode
    {
    public:
    BattlefieldInformationAction(
      const std::string & name, const BT::NodeConfig & config);
    
      static BT::PortsList providedPorts();
    
      BT::NodeStatus tick() override;

    private:
      rclcpp::Logger logger_;
    };


    
}  // namespace sentry_behavior



#endif  //SENTRY_BEHAVIOR__PLUGINS__ACTION__BATTLEFIELD_INFORMATION_HPP_