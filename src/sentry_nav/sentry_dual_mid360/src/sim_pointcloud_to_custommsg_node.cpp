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

#include "pointcloud_merger/sim_pointcloud_to_custommsg_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace sentry_dual_mid360
{

SimPointCloudToCustomMsgNode::SimPointCloudToCustomMsgNode(const rclcpp::NodeOptions & options)
: Node("sim_pointcloud_to_custommsg_node", options)
  , lidar_id_(0)
  , line_count_(4)
  , scan_period_s_(0.1)
  , reflectivity_(10)
  , tag_(0x10)
  , has_last_published_stamp_(false)
  , msg_count_(0)
  , drop_nonfinite_count_(0)
  , monotonic_nudge_count_(0)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "livox/lidar_front_points");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "livox/lidar_front");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "front_mid360");
  lidar_id_ = this->declare_parameter<int>("lidar_id", lidar_id_);
  line_count_ = this->declare_parameter<int>("line_count", line_count_);
  scan_period_s_ = this->declare_parameter<double>("scan_period_s", scan_period_s_);
  reflectivity_ = this->declare_parameter<int>("reflectivity", reflectivity_);
  tag_ = this->declare_parameter<int>("tag", tag_);

  if (line_count_ < 1) {
    RCLCPP_WARN(
      this->get_logger(), "line_count=%d invalid, clamping to 1", line_count_);
    line_count_ = 1;
  }
  if (scan_period_s_ <= 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "scan_period_s=%.6f invalid, clamping to 0.1",
      scan_period_s_);
    scan_period_s_ = 0.1;
  }
  if (reflectivity_ < 0 || reflectivity_ > 255) {
    RCLCPP_WARN(
      this->get_logger(), "reflectivity=%d out of uint8 range, clamping", reflectivity_);
    reflectivity_ = std::clamp(reflectivity_, 0, 255);
  }
  if (tag_ < 0 || tag_ > 255) {
    RCLCPP_WARN(this->get_logger(), "tag=%d out of uint8 range, clamping", tag_);
    tag_ = std::clamp(tag_, 0, 255);
  }

  custom_pub_ = this->create_publisher<CustomMsg>(output_topic_, rclcpp::SensorDataQoS());
  cloud_sub_ = this->create_subscription<PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SimPointCloudToCustomMsgNode::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "sim_pointcloud_to_custommsg_node started: in=%s out=%s frame_id=%s lidar_id=%d "
    "line_count=%d scan_period_s=%.6f reflectivity=%d tag=0x%02x",
    input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(), lidar_id_, line_count_,
    scan_period_s_, reflectivity_, tag_);
}

void SimPointCloudToCustomMsgNode::cloudCallback(const PointCloud2::SharedPtr msg)
{
  if (msg == nullptr) {
    return;
  }

  // Gazebo PointCloudPacked → sensor_msgs::PointCloud2 exposes x/y/z as float32.
  // Use PointCloud2ConstIterator to decode without pulling in PCL, and keep only
  // finite points to avoid feeding Point-LIO NaN/Inf from sensor noise edges.
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  const size_t input_point_count =
    static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (input_point_count == 0) {
    return;
  }

  CustomMsg output_msg;
  output_msg.header = msg->header;
  output_msg.header.frame_id = frame_id_;
  rclcpp::Time stamp(msg->header.stamp);
  if (has_last_published_stamp_ && stamp <= last_published_stamp_) {
    stamp = last_published_stamp_ + rclcpp::Duration(0, 1);
    output_msg.header.stamp = stamp;
    ++monotonic_nudge_count_;
  }
  output_msg.timebase = 0;
  output_msg.lidar_id = static_cast<uint8_t>(std::clamp(lidar_id_, 0, 255));
  output_msg.rsvd = {0, 0, 0};
  output_msg.points.reserve(input_point_count);

  const double scan_period_ns_d =
    std::max(0.0, scan_period_s_) * 1e9;
  const uint64_t scan_period_ns = static_cast<uint64_t>(
    std::min(
      scan_period_ns_d,
      static_cast<double>(std::numeric_limits<uint32_t>::max())));
  const uint32_t line_count_u32 = static_cast<uint32_t>(line_count_);

  size_t accepted_index = 0;
  for (size_t i = 0; i < input_point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      ++drop_nonfinite_count_;
      continue;
    }

    CustomPoint point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.reflectivity = static_cast<uint8_t>(reflectivity_);
    point.tag = static_cast<uint8_t>(tag_);
    point.line = static_cast<uint8_t>(accepted_index % line_count_u32);

    // Linear ramp over scan_period — spreads points uniformly so the sort on
    // offset_time is stable and matches MergerNode's expectation that offsets
    // fit in uint32 ns.
    uint64_t offset_ns = 0;
    if (input_point_count > 1) {
      offset_ns = (static_cast<uint64_t>(accepted_index) * scan_period_ns) /
        static_cast<uint64_t>(input_point_count - 1);
    }
    if (offset_ns > std::numeric_limits<uint32_t>::max()) {
      offset_ns = std::numeric_limits<uint32_t>::max();
    }
    point.offset_time = static_cast<uint32_t>(offset_ns);

    output_msg.points.push_back(point);
    ++accepted_index;
  }

  output_msg.point_num = static_cast<uint32_t>(output_msg.points.size());
  if (output_msg.point_num == 0) {
    return;
  }

  custom_pub_->publish(std::move(output_msg));
  last_published_stamp_ = stamp;
  has_last_published_stamp_ = true;
  ++msg_count_;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Sim PC2→CustomMsg metrics: published=%lu drop_nonfinite=%lu stamp_nudges=%lu "
    "last_points=%zu frame_id=%s",
    msg_count_, drop_nonfinite_count_, monotonic_nudge_count_, accepted_index,
    frame_id_.c_str());
}

}  // namespace sentry_dual_mid360

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_dual_mid360::SimPointCloudToCustomMsgNode>());
  rclcpp::shutdown();
  return 0;
}
