#include "sentry_behavior/plugins/action/pursuit.hpp"

namespace pursuit_bt
{

PursuitBTNode::PursuitBTNode(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params),
  tf_buffer_(this->node_->get_clock()),
  tf_listener_(tf_buffer_)
{
  // enemy_point_sub_ = node_->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "enemy_pose", 10, std::bind(&PursuitBTNode::receiveEnemyPoint, this, std::placeholders::_1));

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
}

BT::PortsList PursuitBTNode::providedPorts()
{
  return {
    BT::InputPort<auto_aim_interfaces::msg::Target>(
      "key_port", "{@tracker_target}", "target port on blackboard"),
    BT::InputPort<std::string>("topic_name"),
  };
}

bool PursuitBTNode::setMessage(geometry_msgs::msg::PoseStamped & goal)
{
  if (!nav2_util::getCurrentPose(robot_pose_, tf_buffer_, "map", "chassis", 0.5)) {
    RCLCPP_WARN(node_->get_logger(), "Failed to get robot pose");
    return false;
  }
  auto msg = getInput<auto_aim_interfaces::msg::Target>("key_port");
  if (!msg) {
    RCLCPP_ERROR(node_->get_logger(), "allRobotHP message is not available");
    return false;
  } 
  auto target=msg.value();
  target_.position.x= target.position.x;
  target_.position.y= target.position.y;
  calculatePursuitPose(target_);
  publishMarkers();

  goal = pursuit_pose;
  return true;
}

// void PursuitBTNode::receiveEnemyPoint(const auto_aim_interfaces::msg::Target & msg)
// {
//   target_ = msg;
//   RCLCPP_INFO(node_->get_logger(), "Received enemy point: x=%f, y=%f", msg.position.x, msg.position.y);
// }

void PursuitBTNode::calculatePursuitPose(const auto_aim_interfaces::msg::Target & target_)
{
  double self_x = robot_pose_.pose.position.x;
  double self_y = robot_pose_.pose.position.y;
  double enemy_x = self_x + target_.position.x;
  double enemy_y = self_y + target_.position.y;

  
  double dx = self_x - enemy_x;
  double dy = self_y - enemy_y;
  double distance = std::hypot(dx, dy);
  
  double safe_distance = 2.0;
  double ratio = (distance < 0.2) ? 1.0 : (safe_distance / distance);

  if (distance < 2.0) {
    
    pursuit_pose.pose.position.x = enemy_x + dx * ratio;
    pursuit_pose.pose.position.y = enemy_y + dy * ratio;
  } else if(distance>2.0){
    pursuit_pose.pose.position.x = enemy_x ;
    pursuit_pose.pose.position.y = enemy_y ;
  }else{
    
    pursuit_pose.pose.position.x = self_x;
    pursuit_pose.pose.position.y = self_y;
  }

  pursuit_pose.header.stamp = node_->now();
  pursuit_pose.header.frame_id = "map";
}

void PursuitBTNode::publishMarkers()
{
  visualization_msgs::msg::Marker enemy_marker;
  visualization_msgs::msg::Marker pursuit_marker;

  enemy_marker.header.stamp = node_->now();
  enemy_marker.header.frame_id = "map";
  enemy_marker.ns = "enemy_pose";
  enemy_marker.id = 0;
  enemy_marker.type = visualization_msgs::msg::Marker::SPHERE;
  enemy_marker.action = visualization_msgs::msg::Marker::ADD;
  enemy_marker.pose = robot_pose_.pose;
  enemy_marker.pose.position.z = 0.35;
  enemy_marker.scale.x = 0.3;
  enemy_marker.scale.y = 0.3;
  enemy_marker.scale.z = 0.3;
  enemy_marker.color.r = 1.0;
  enemy_marker.color.g = 0.64;
  enemy_marker.color.b = 0.0;
  enemy_marker.color.a = 1.0;
  enemy_marker.lifetime = rclcpp::Duration::from_seconds(0.15);

  pursuit_marker.header.stamp = node_->now();
  pursuit_marker.header.frame_id = "map";
  pursuit_marker.ns = "pursuit_pose";
  pursuit_marker.id = 1;
  pursuit_marker.type = visualization_msgs::msg::Marker::SPHERE;
  pursuit_marker.action = visualization_msgs::msg::Marker::ADD;
  pursuit_marker.pose = pursuit_pose.pose;
  pursuit_marker.pose.position.z = 0.35;
  pursuit_marker.scale.x = 0.2;
  pursuit_marker.scale.y = 0.2;
  pursuit_marker.scale.z = 0.2;
  pursuit_marker.color.r = 0.96;
  pursuit_marker.color.g = 0.87;
  pursuit_marker.color.b = 0.7;
  pursuit_marker.color.a = 1.0;
  pursuit_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  marker_pub_->publish(enemy_marker);
  marker_pub_->publish(pursuit_marker);
}

}  // namespace pursuit_bt

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pursuit_bt::PursuitBTNode, "Pursuit");