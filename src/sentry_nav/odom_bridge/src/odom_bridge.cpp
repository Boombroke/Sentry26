#include "odom_bridge/odom_bridge.hpp"

#include <algorithm>
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
  previous_stamp_(0, 0, RCL_ROS_TIME),
  has_previous_twist_(false),
  min_twist_dt_(1e-3),
  max_twist_dt_(0.1),
  max_valid_linear_speed_(3.0),
  max_valid_lateral_speed_(0.35),
  max_valid_angular_speed_(6.3),
  max_valid_linear_accel_(12.0),
  max_valid_angular_accel_(40.0),
  twist_filter_alpha_(0.45),
  flip_roll_threshold_(0.5236),
  flip_pitch_threshold_(0.5236)
{
  this->declare_parameter<std::string>("state_estimation_topic", "aft_mapped_to_init");
  this->declare_parameter<std::string>("registered_scan_topic", "cloud_registered");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("lidar_frame", "front_mid360");
  this->declare_parameter<std::string>("robot_base_frame", "base_footprint");
  this->declare_parameter<double>("min_twist_dt", min_twist_dt_);
  this->declare_parameter<double>("max_twist_dt", max_twist_dt_);
  this->declare_parameter<double>("max_valid_linear_speed", max_valid_linear_speed_);
  this->declare_parameter<double>("max_valid_lateral_speed", max_valid_lateral_speed_);
  this->declare_parameter<double>("max_valid_angular_speed", max_valid_angular_speed_);
  this->declare_parameter<double>("max_valid_linear_accel", max_valid_linear_accel_);
  this->declare_parameter<double>("max_valid_angular_accel", max_valid_angular_accel_);
  this->declare_parameter<double>("twist_filter_alpha", twist_filter_alpha_);
  this->declare_parameter<double>("flip_roll_threshold", 0.5236);   // 30 deg
  this->declare_parameter<double>("flip_pitch_threshold", 0.5236);  // 30 deg

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("min_twist_dt", min_twist_dt_);
  this->get_parameter("max_twist_dt", max_twist_dt_);
  this->get_parameter("max_valid_linear_speed", max_valid_linear_speed_);
  this->get_parameter("max_valid_lateral_speed", max_valid_lateral_speed_);
  this->get_parameter("max_valid_angular_speed", max_valid_angular_speed_);
  this->get_parameter("max_valid_linear_accel", max_valid_linear_accel_);
  this->get_parameter("max_valid_angular_accel", max_valid_angular_accel_);
  this->get_parameter("twist_filter_alpha", twist_filter_alpha_);
  this->get_parameter("flip_roll_threshold", flip_roll_threshold_);
  this->get_parameter("flip_pitch_threshold", flip_pitch_threshold_);
  twist_filter_alpha_ = std::clamp(twist_filter_alpha_, 0.0, 1.0);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sensor_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);
  registered_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  lidar_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);
  robot_flipped_pub_ = this->create_publisher<std_msgs::msg::Bool>("robot_flipped", 2);

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
  // 翻车检测：从 raw 变换提取底盘真实 roll/pitch（与雷达安装角无关，外参已在 TF 链中抵消）
  {
    tf2::Quaternion q_raw = tf_odom_to_robot_base_raw.getRotation();
    q_raw.normalize();
    double raw_roll;
    double raw_pitch;
    double raw_yaw_unused;
    tf2::Matrix3x3(q_raw).getRPY(raw_roll, raw_pitch, raw_yaw_unused);

    const bool is_flipped =
      (std::abs(raw_roll) > flip_roll_threshold_) ||
      (std::abs(raw_pitch) > flip_pitch_threshold_);

    std_msgs::msg::Bool flipped_msg;
    flipped_msg.data = is_flipped;
    robot_flipped_pub_->publish(flipped_msg);

    if (is_flipped) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Robot flip detected: roll=%.3f rad (%.1f deg), pitch=%.3f rad (%.1f deg)",
        raw_roll, raw_roll * 180.0 / M_PI, raw_pitch, raw_pitch * 180.0 / M_PI);
    }
  }
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

    // dt 窗口收紧: 低于 min_twist_dt 是重复戳; 高于 max_twist_dt 是 LIO burst 空窗.
    // 两种情况都保持 twist=0, 避免跨多帧差分制造虚假速度尖峰.
    if (dt > min_twist_dt_ && dt < max_twist_dt_) {
      const tf2::Vector3 v_world =
        (transform.getOrigin() - previous_transform_.getOrigin()) / dt;

      // 对外语义固定为地面移动底盘: pose 已在调用侧压成 z=0, roll=0, pitch=0.
      // 这里再按平面 yaw 把世界系速度投到底盘系, 避免传感器自身 roll/pitch 通过
      // odometry 回灌成虚假的 vy / wx / wy.
      tf2::Quaternion q = transform.getRotation();
      q.normalize();
      const tf2::Matrix3x3 R(q);
      const tf2::Vector3 v_body = R.transpose() * v_world;

      const double wz = normalizedYawDelta(previous_transform_.getRotation(), q) / dt;
      const bool speed_ok =
        std::abs(v_body.x()) <= max_valid_linear_speed_ &&
        std::abs(v_body.y()) <= max_valid_lateral_speed_ &&
        std::abs(wz) <= max_valid_angular_speed_;
      const bool accel_ok =
        !has_previous_twist_ ||
        (std::abs(v_body.x() - previous_twist_.linear.x) / dt <= max_valid_linear_accel_ &&
        std::abs(wz - previous_twist_.angular.z) / dt <= max_valid_angular_accel_);

      if (speed_ok && accel_ok) {
        geometry_msgs::msg::Twist raw_twist;
        raw_twist.linear.x = v_body.x();
        raw_twist.linear.y = 0.0;
        raw_twist.linear.z = 0.0;
        raw_twist.angular.x = 0.0;
        raw_twist.angular.y = 0.0;
        raw_twist.angular.z = wz;

        if (has_previous_twist_) {
          out.twist.twist.linear.x =
            twist_filter_alpha_ * raw_twist.linear.x +
            (1.0 - twist_filter_alpha_) * previous_twist_.linear.x;
          out.twist.twist.angular.z =
            twist_filter_alpha_ * raw_twist.angular.z +
            (1.0 - twist_filter_alpha_) * previous_twist_.angular.z;
        } else {
          out.twist.twist = raw_twist;
        }
        previous_twist_ = out.twist.twist;
        has_previous_twist_ = true;
      } else if (has_previous_twist_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Rejected odom twist spike: vx=%.3f vy=%.3f wz=%.3f dt=%.3f speed_ok=%d accel_ok=%d",
          v_body.x(), v_body.y(), wz, dt, speed_ok, accel_ok);
        has_previous_twist_ = false;
      }
    } else {
      has_previous_twist_ = false;
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
