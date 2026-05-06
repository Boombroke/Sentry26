#ifndef SENTRY_DUAL_MID360__POINTCLOUD_MERGER__MERGER_NODE_HPP_
#define SENTRY_DUAL_MID360__POINTCLOUD_MERGER__MERGER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "Eigen/Geometry"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sentry_dual_mid360
{

class MergerNode : public rclcpp::Node
{
public:
  explicit MergerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  using CustomPoint = livox_ros_driver2::msg::CustomPoint;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<CustomMsg, CustomMsg>;

  void mergeCallback(
    const CustomMsg::ConstSharedPtr & front_msg, const CustomMsg::ConstSharedPtr & back_msg);

  bool ensureBackToCommonTransform();
  bool appendPointWithRebasedOffset(
    const CustomPoint & point, uint64_t rebase_offset_ns, CustomMsg & output_msg,
    uint64_t & drop_overflow_count) const;
  static bool isBeyondMinDistance(const CustomPoint & point, double min_distance_m);

  std::string front_topic_;
  std::string back_topic_;
  std::string output_topic_;
  std::string common_frame_;
  std::string back_frame_;
  double sync_tolerance_ms_;
  double min_dist_front_m_;
  double min_dist_back_m_;
  int queue_size_;

  rclcpp::Publisher<CustomMsg>::SharedPtr merged_pub_;
  message_filters::Subscriber<CustomMsg> front_sub_;
  message_filters::Subscriber<CustomMsg> back_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  bool has_back_to_common_transform_;
  Eigen::Isometry3d back_to_common_transform_;

  rclcpp::Time last_published_stamp_;
  bool has_last_published_stamp_;
  uint64_t msg_count_;
  uint64_t drop_tf_;
  uint64_t drop_sync_;
  uint64_t drop_front_min_dist_;
  uint64_t drop_back_min_dist_;
  uint64_t drop_front_offset_overflow_;
  uint64_t drop_back_offset_overflow_;
  uint64_t monotonic_nudge_count_;
};

}  // namespace sentry_dual_mid360

#endif  // SENTRY_DUAL_MID360__POINTCLOUD_MERGER__MERGER_NODE_HPP_
