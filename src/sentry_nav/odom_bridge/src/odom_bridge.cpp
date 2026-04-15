#include "odom_bridge/odom_bridge.hpp"

#include <cmath>

#include "pcl_ros/transforms.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace odom_bridge
{

OdomBridgeNode::OdomBridgeNode(const rclcpp::NodeOptions & options)
: Node("odom_bridge", options),
  base_frame_to_lidar_initialized_(false),
  tf_odom_to_lidar_odom_(tf2::Transform::getIdentity()),
  has_previous_transform_(false),
  previous_transform_(tf2::Transform::getIdentity()),
  previous_time_(std::chrono::steady_clock::time_point::min())
{
  this->declare_parameter<std::string>("state_estimation_topic", "aft_mapped_to_init");
  this->declare_parameter<std::string>("registered_scan_topic", "cloud_registered");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("lidar_frame", "front_mid360");
  this->declare_parameter<std::string>("robot_base_frame", "gimbal_yaw");

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sensor_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);
  registered_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  lidar_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  odom_to_lidar_odom_pub_ =
    this->create_publisher<geometry_msgs::msg::TransformStamped>("odom_to_lidar_odom", latched_qos);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.depth = 5;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;

  odometry_sub_.subscribe(this, state_estimation_topic_, qos_profile);
  registered_scan_sub_.subscribe(this, registered_scan_topic_, qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, registered_scan_sub_);
  sync_->registerCallback(std::bind(
    &OdomBridgeNode::lidarOdometryAndPointCloudCallback, this,
    std::placeholders::_1, std::placeholders::_2));
}

void OdomBridgeNode::lidarOdometryAndPointCloudCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  if (!base_frame_to_lidar_initialized_) {
    try {
      const auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, odometry_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.5));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);

      // Point-LIO first frame pose = rot_init (gravity alignment rotation).
      // lidar_odom frame is rotated by rot_init relative to the physical lidar frame at t=0.
      // Compensate: odom→lidar_odom = (base→lidar) * rot_init_inverse
      tf2::Transform tf_lidar_odom_to_lidar_t0;
      tf2::fromMsg(odometry_msg->pose.pose, tf_lidar_odom_to_lidar_t0);
      tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar * tf_lidar_odom_to_lidar_t0.inverse();

      base_frame_to_lidar_initialized_ = true;

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = odometry_msg->header.stamp;
      msg.header.frame_id = odom_frame_;
      msg.child_frame_id = "lidar_odom";
      msg.transform = tf2::toMsg(tf_odom_to_lidar_odom_);
      odom_to_lidar_odom_pub_->publish(msg);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Retrying...", ex.what());
      return;
    }
  }

  sensor_msgs::msg::PointCloud2 registered_scan_in_odom;
  pcl_ros::transformPointCloud(
    odom_frame_, tf_odom_to_lidar_odom_, *pcd_msg, registered_scan_in_odom);

  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, tf_lidar_odom_to_lidar);
  const tf2::Transform tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;

  const tf2::Transform tf_lidar_to_chassis =
    getTransform(lidar_frame_, base_frame_, pcd_msg->header.stamp);
  const tf2::Transform tf_lidar_to_robot_base =
    getTransform(lidar_frame_, robot_base_frame_, pcd_msg->header.stamp);

  tf2::Transform tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  const tf2::Transform tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base;

  {
    const auto & origin = tf_odom_to_chassis.getOrigin();
    tf2::Quaternion q = tf_odom_to_chassis.getRotation();
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    tf_odom_to_chassis.setOrigin(tf2::Vector3(origin.x(), origin.y(), 0.0));
    tf2::Quaternion q_2d;
    q_2d.setRPY(0.0, 0.0, yaw);
    tf_odom_to_chassis.setRotation(q_2d);
  }

  publishTransform(tf_odom_to_chassis, odom_frame_, base_frame_, pcd_msg->header.stamp);
  publishOdometry(tf_odom_to_robot_base, odom_frame_, robot_base_frame_, pcd_msg->header.stamp);

  sensor_msgs::msg::PointCloud2 sensor_scan;
  pcl_ros::transformPointCloud(
    lidar_frame_, tf_odom_to_lidar.inverse(), registered_scan_in_odom, sensor_scan);
  sensor_scan_pub_->publish(sensor_scan);

  registered_scan_pub_->publish(registered_scan_in_odom);
  {
    nav_msgs::msg::Odometry lidar_odom_out;
    lidar_odom_out.header.stamp = pcd_msg->header.stamp;
    lidar_odom_out.header.frame_id = odom_frame_;
    lidar_odom_out.child_frame_id = lidar_frame_;
    const auto & origin = tf_odom_to_lidar.getOrigin();
    lidar_odom_out.pose.pose.position.x = origin.x();
    lidar_odom_out.pose.pose.position.y = origin.y();
    lidar_odom_out.pose.pose.position.z = origin.z();
    lidar_odom_out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());
    lidar_odometry_pub_->publish(lidar_odom_out);
  }
}

tf2::Transform OdomBridgeNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    const auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
}

void OdomBridgeNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  tf_broadcaster_->sendTransform(transform_msg);
}

void OdomBridgeNode::publishOdometry(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (has_previous_transform_) {
    const auto current_time = std::chrono::steady_clock::now();
    const double dt =
      std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - previous_time_).count() *
      1e-9;

    if (dt > 0.0) {
      const auto linear_velocity = (transform.getOrigin() - previous_transform_.getOrigin()) / dt;
      const tf2::Quaternion q_diff =
        transform.getRotation() * previous_transform_.getRotation().inverse();
      const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

      out.twist.twist.linear.x = linear_velocity.x();
      out.twist.twist.linear.y = linear_velocity.y();
      out.twist.twist.linear.z = linear_velocity.z();
      out.twist.twist.angular.x = angular_velocity.x();
      out.twist.twist.angular.y = angular_velocity.y();
      out.twist.twist.angular.z = angular_velocity.z();
    }

    previous_transform_ = transform;
    previous_time_ = current_time;
  } else {
    previous_transform_ = transform;
    previous_time_ = std::chrono::steady_clock::now();
    has_previous_transform_ = true;
  }

  odometry_pub_->publish(out);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(odom_bridge::OdomBridgeNode)
