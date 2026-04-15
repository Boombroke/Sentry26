#ifndef ODOM_BRIDGE__ODOM_BRIDGE_HPP_
#define ODOM_BRIDGE__ODOM_BRIDGE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace odom_bridge
{

class OdomBridgeNode : public rclcpp::Node
{
public:
  explicit OdomBridgeNode(const rclcpp::NodeOptions & options);

private:
  void lidarOdometryAndPointCloudCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg);

  tf2::Transform getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishOdometry(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  std::string state_estimation_topic_;
  std::string registered_scan_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string robot_base_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom_to_lidar_odom_pub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> registered_scan_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  bool base_frame_to_lidar_initialized_;
  tf2::Transform tf_odom_to_lidar_odom_;

  bool has_previous_transform_;
  tf2::Transform previous_transform_;
  std::chrono::steady_clock::time_point previous_time_;
};

}

#endif
