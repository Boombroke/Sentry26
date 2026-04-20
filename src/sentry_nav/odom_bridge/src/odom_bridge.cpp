#include "odom_bridge/odom_bridge.hpp"

#include <cmath>

#include "pcl_ros/transforms.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace odom_bridge
{

namespace
{

tf2::Transform projectToPlanarBase(const tf2::Transform & transform)
{
  tf2::Transform projected = transform;
  const auto & origin = transform.getOrigin();
  tf2::Quaternion q = transform.getRotation();
  q.normalize();

  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  projected.setOrigin(tf2::Vector3(origin.x(), origin.y(), 0.0));
  tf2::Quaternion q_2d;
  q_2d.setRPY(0.0, 0.0, yaw);
  projected.setRotation(q_2d);
  return projected;
}

double normalizedYawDelta(const tf2::Quaternion & from, const tf2::Quaternion & to)
{
  tf2::Quaternion q_from = from;
  tf2::Quaternion q_to = to;
  q_from.normalize();
  q_to.normalize();

  double from_roll;
  double from_pitch;
  double from_yaw;
  tf2::Matrix3x3(q_from).getRPY(from_roll, from_pitch, from_yaw);

  double to_roll;
  double to_pitch;
  double to_yaw;
  tf2::Matrix3x3(q_to).getRPY(to_roll, to_pitch, to_yaw);

  return std::atan2(std::sin(to_yaw - from_yaw), std::cos(to_yaw - from_yaw));
}

}  // namespace

OdomBridgeNode::OdomBridgeNode(const rclcpp::NodeOptions & options)
: Node("odom_bridge", options),
  base_frame_to_lidar_initialized_(false),
  tf_odom_to_lidar_odom_(tf2::Transform::getIdentity()),
  has_previous_transform_(false),
  previous_transform_(tf2::Transform::getIdentity()),
  previous_stamp_(0, 0, RCL_ROS_TIME)
{
  this->declare_parameter<std::string>("state_estimation_topic", "aft_mapped_to_init");
  this->declare_parameter<std::string>("registered_scan_topic", "cloud_registered");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("lidar_frame", "front_mid360");
  this->declare_parameter<std::string>("robot_base_frame", "base_footprint");

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

  // 用 odometry_msg 的时间戳查 TF (保证与位姿同一时刻, 避免 approximate sync
  // 带来的几毫秒偏差在云台高速旋转时放大成虚假位移)
  const rclcpp::Time pose_stamp(odometry_msg->header.stamp, RCL_ROS_TIME);

  const tf2::Transform tf_lidar_to_chassis =
    getTransform(lidar_frame_, base_frame_, pose_stamp);
  const tf2::Transform tf_lidar_to_robot_base =
    getTransform(lidar_frame_, robot_base_frame_, pose_stamp);

  tf2::Transform tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  const tf2::Transform tf_odom_to_robot_base_raw = tf_odom_to_lidar * tf_lidar_to_robot_base;
  const tf2::Transform tf_odom_to_robot_base = projectToPlanarBase(tf_odom_to_robot_base_raw);
  tf_odom_to_chassis = projectToPlanarBase(tf_odom_to_chassis);

  publishTransform(tf_odom_to_chassis, odom_frame_, base_frame_, pose_stamp);
  publishOdometry(tf_odom_to_robot_base, odom_frame_, robot_base_frame_, pose_stamp);

  sensor_msgs::msg::PointCloud2 sensor_scan;
  pcl_ros::transformPointCloud(
    lidar_frame_, tf_odom_to_lidar.inverse(), registered_scan_in_odom, sensor_scan);
  sensor_scan_pub_->publish(sensor_scan);

  registered_scan_pub_->publish(registered_scan_in_odom);
  {
    nav_msgs::msg::Odometry lidar_odom_out;
    lidar_odom_out.header.stamp = pose_stamp;
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
    // 用消息时间戳（sim_time）而非 wall clock 算 dt,
    // 否则仿真环境下 wall_time 和 sim_time 不同步会爆飞速度值
    const double dt = (stamp - previous_stamp_).seconds();

    // dt 窗口收紧: 低于 1ms 是重复戳; 高于 0.1s 是 LIO burst 空窗, 跨多帧差分会爆飞.
    // 两种情况都保持 twist=0, 让 Nav2 velocity_smoother 用上一拍反馈, 不输出虚假速度.
    if (dt > 1e-3 && dt < 0.1) {
      const tf2::Vector3 v_world =
        (transform.getOrigin() - previous_transform_.getOrigin()) / dt;

      // 对外语义固定为地面移动底盘: pose 已在调用侧压成 z=0, roll=0, pitch=0.
      // 这里再按平面 yaw 把世界系速度投到底盘系, 避免传感器自身 roll/pitch 通过
      // odometry 回灌成虚假的 vy / wx / wy.
      tf2::Quaternion q = transform.getRotation();
      q.normalize();
      const tf2::Matrix3x3 R(q);
      const tf2::Vector3 v_body = R.transpose() * v_world;

      // body 速度 sanity check: 差速底盘物理上限 ~3 m/s, >10 m/s 必然是 LIO 位姿抖动,
      // 丢弃避免 velocity_smoother 把污染值当反馈回灌给 controller.
      if (std::abs(v_body.x()) < 10.0 && std::abs(v_body.y()) < 10.0) {
        out.twist.twist.linear.x = v_body.x();
        out.twist.twist.linear.y = 0.0;
        out.twist.twist.linear.z = 0.0;
        out.twist.twist.angular.x = 0.0;
        out.twist.twist.angular.y = 0.0;
        out.twist.twist.angular.z =
          normalizedYawDelta(previous_transform_.getRotation(), q) / dt;
      }
    }

    previous_transform_ = transform;
    previous_stamp_ = stamp;
  } else {
    previous_transform_ = transform;
    previous_stamp_ = stamp;
    has_previous_transform_ = true;
  }

  odometry_pub_->publish(out);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(odom_bridge::OdomBridgeNode)
