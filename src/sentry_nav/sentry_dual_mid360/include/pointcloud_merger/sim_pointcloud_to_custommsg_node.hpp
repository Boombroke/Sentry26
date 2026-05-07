// Copyright 2026 Boombroke
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

#ifndef SENTRY_DUAL_MID360__POINTCLOUD_MERGER__SIM_POINTCLOUD_TO_CUSTOMMSG_NODE_HPP_
#define SENTRY_DUAL_MID360__POINTCLOUD_MERGER__SIM_POINTCLOUD_TO_CUSTOMMSG_NODE_HPP_

#include <cstdint>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace sentry_dual_mid360
{

// Convert Gazebo gpu_lidar PointCloud2 into Livox CustomMsg so that the sim
// dual-Mid360 chain can reuse the real-robot MergerNode and keep Point-LIO as a
// black box. One converter node per simulated Mid360 (front/back).
class SimPointCloudToCustomMsgNode : public rclcpp::Node
{
public:
  explicit SimPointCloudToCustomMsgNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  using CustomPoint = livox_ros_driver2::msg::CustomPoint;

  void cloudCallback(const PointCloud2::SharedPtr msg);

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int lidar_id_;
  int line_count_;
  double scan_period_s_;
  int reflectivity_;
  int tag_;

  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<CustomMsg>::SharedPtr custom_pub_;

  rclcpp::Time last_published_stamp_;
  bool has_last_published_stamp_;
  uint64_t msg_count_;
  uint64_t drop_nonfinite_count_;
  uint64_t monotonic_nudge_count_;
};

}  // namespace sentry_dual_mid360

#endif  // SENTRY_DUAL_MID360__POINTCLOUD_MERGER__SIM_POINTCLOUD_TO_CUSTOMMSG_NODE_HPP_
