#ifndef PURSUIT_BT_NODE_HPP_
#define PURSUIT_BT_NODE_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/robot_utils.hpp"

namespace pursuit_bt
{
class PursuitBTNode : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
  PursuitBTNode(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(geometry_msgs::msg::PoseStamped & goal) override;

private:
  //void receiveEnemyPoint(const auto_aim_interfaces::msg::Target & msg);
  void calculatePursuitPose(const auto_aim_interfaces::msg::Target & target_);
  void publishMarkers();

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr enemy_point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  geometry_msgs::msg::PoseStamped robot_pose_;
  geometry_msgs::msg::PoseStamped pursuit_pose;
  auto_aim_interfaces::msg::Target target_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace pursuit_bt

#endif  // PURSUIT_BT_NODE_HPP_