// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<float>("init_spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(angle_mutex_);
  current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double angle;
  {
    std::lock_guard<std::mutex> lock(angle_mutex_);
    angle = current_robot_base_angle_;
  }
  auto aft_tf_vel = transformVelocity(msg, angle);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::publishTransform()
{
  double angle;
  {
    std::lock_guard<std::mutex> lock(angle_mutex_);
    angle = current_robot_base_angle_;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -angle);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
