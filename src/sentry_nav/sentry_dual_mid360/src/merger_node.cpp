#include "pointcloud_merger/merger_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace
{

constexpr uint64_t kNanosecondsPerMillisecond = 1000000ULL;

int64_t stampDiffNanoseconds(const rclcpp::Time & lhs, const rclcpp::Time & rhs)
{
  return lhs.nanoseconds() - rhs.nanoseconds();
}

uint64_t absoluteNanoseconds(int64_t value)
{
  if (value >= 0) {
    return static_cast<uint64_t>(value);
  }
  return static_cast<uint64_t>(-(value + 1)) + 1ULL;
}

}  // namespace

namespace sentry_dual_mid360
{

MergerNode::MergerNode(const rclcpp::NodeOptions & options)
: Node("merger_node", options)
  , sync_tolerance_ms_(10.0)
  , min_dist_primary_m_(0.4)
  , min_dist_secondary_m_(0.4)
  , queue_size_(10)
  , tf_cache_retry_count_(10)
  , tf_cache_retry_interval_ms_(500)
  , has_secondary_to_common_transform_(false)
  , secondary_to_common_transform_(Eigen::Isometry3d::Identity())
  , has_last_published_stamp_(false)
  , msg_count_(0)
  , drop_tf_(0)
  , drop_sync_(0)
  , drop_primary_min_dist_(0)
  , drop_secondary_min_dist_(0)
  , drop_primary_offset_overflow_(0)
  , drop_secondary_offset_overflow_(0)
  , monotonic_nudge_count_(0)
{
  primary_topic_ = this->declare_parameter<std::string>("primary_topic", "/livox/lidar_primary");
  secondary_topic_ =
    this->declare_parameter<std::string>("secondary_topic", "/livox/lidar_secondary");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/lidar");
  pc2_preview_topic_ =
    this->declare_parameter<std::string>("pc2_preview_topic", "/livox/lidar_pc2");
  common_frame_ = this->declare_parameter<std::string>("common_frame", "primary_mid360");
  secondary_frame_ =
    this->declare_parameter<std::string>("secondary_frame", "secondary_mid360");
  sync_tolerance_ms_ = this->declare_parameter<double>("sync_tolerance_ms", sync_tolerance_ms_);
  min_dist_primary_m_ =
    this->declare_parameter<double>("min_dist_primary_m", min_dist_primary_m_);
  min_dist_secondary_m_ =
    this->declare_parameter<double>("min_dist_secondary_m", min_dist_secondary_m_);
  queue_size_ = this->declare_parameter<int>("queue_size", queue_size_);
  // Declared for YAML visibility; T8 retries TF every sync callback (no cadence use yet).
  tf_cache_retry_count_ =
    this->declare_parameter<int>("tf_cache_retry_count", tf_cache_retry_count_);
  tf_cache_retry_interval_ms_ =
    this->declare_parameter<int>("tf_cache_retry_interval_ms", tf_cache_retry_interval_ms_);
  // Optional PointCloud2 copy of the merged output, for rviz visualization.
  // Off by default — production consumers (Point-LIO) want CustomMsg. Enable
  // when debugging extrinsic calibration without the full bringup stack.
  publish_pc2_preview_ =
    this->declare_parameter<bool>("publish_pc2_preview", false);
  if (sync_tolerance_ms_ < 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "sync_tolerance_ms=%.3f is invalid, clamping to 0.0",
      sync_tolerance_ms_);
    sync_tolerance_ms_ = 0.0;
  }
  if (queue_size_ < 1) {
    RCLCPP_WARN(
      this->get_logger(), "queue_size=%d is invalid, clamping to 1", queue_size_);
    queue_size_ = 1;
  }
  if (tf_cache_retry_count_ < 0) {
    RCLCPP_WARN(
      this->get_logger(), "tf_cache_retry_count=%d is invalid, clamping to 0",
      tf_cache_retry_count_);
    tf_cache_retry_count_ = 0;
  }
  if (tf_cache_retry_interval_ms_ < 0) {
    RCLCPP_WARN(
      this->get_logger(), "tf_cache_retry_interval_ms=%d is invalid, clamping to 0",
      tf_cache_retry_interval_ms_);
    tf_cache_retry_interval_ms_ = 0;
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  merged_pub_ = this->create_publisher<CustomMsg>(output_topic_, rclcpp::SensorDataQoS());
  if (publish_pc2_preview_) {
    // Publish PC2 mirror on SensorDataQoS (BEST_EFFORT, depth=5) — same
    // profile as the main CustomMsg output. Earlier attempt switched to
    // RELIABLE to match rviz2's default subscription QoS, but that blocks
    // the sync callback thread when rviz is slow to ACK (depth saturates,
    // publish() waits), starving the merger: metrics stopped after
    // published=1 until rviz disconnected.
    //
    // Consequence: the user must flip rviz's PointCloud2 display
    // "Reliability Policy" from "Reliable" to "Best Effort" to subscribe.
    // The alternative — blocking the real-time merger to satisfy rviz —
    // is far worse. This trade-off is documented in lidar_only_debug.sh
    // startup text and CALIBRATION_QUICKSTART.md.
    merged_pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      pc2_preview_topic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(
      this->get_logger(),
      "publish_pc2_preview=true: mirroring merged output to %s (SensorDataQoS / "
      "BEST_EFFORT). In rviz set the PointCloud2 display Reliability to Best Effort.",
      pc2_preview_topic_.c_str());
  }

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.depth = static_cast<size_t>(queue_size_);
  primary_sub_.subscribe(this, primary_topic_, qos_profile);
  secondary_sub_.subscribe(this, secondary_topic_, qos_profile);

  SyncPolicy sync_policy(static_cast<uint32_t>(queue_size_));
  sync_policy.setMaxIntervalDuration(
    rclcpp::Duration::from_seconds(sync_tolerance_ms_ / 1000.0));
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    static_cast<const SyncPolicy &>(sync_policy), primary_sub_, secondary_sub_);
  sync_->registerCallback(std::bind(
    &MergerNode::mergeCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    this->get_logger(),
    "sentry_dual_mid360 merger_node started: primary=%s secondary=%s output=%s common_frame=%s "
    "secondary_frame=%s tolerance=%.3fms queue_size=%d",
    primary_topic_.c_str(), secondary_topic_.c_str(), output_topic_.c_str(),
    common_frame_.c_str(), secondary_frame_.c_str(), sync_tolerance_ms_, queue_size_);
}

void MergerNode::mergeCallback(
  const CustomMsg::ConstSharedPtr & primary_msg, const CustomMsg::ConstSharedPtr & secondary_msg)
{
  if (!ensureSecondaryToCommonTransform()) {
    return;
  }

  const rclcpp::Time primary_stamp(primary_msg->header.stamp);
  const rclcpp::Time secondary_stamp(secondary_msg->header.stamp);
  const int64_t signed_stamp_diff_ns = stampDiffNanoseconds(primary_stamp, secondary_stamp);
  const uint64_t abs_stamp_diff_ns = absoluteNanoseconds(signed_stamp_diff_ns);
  const auto tolerance_ns = static_cast<uint64_t>(sync_tolerance_ms_ * kNanosecondsPerMillisecond);
  if (abs_stamp_diff_ns > tolerance_ns) {
    ++drop_sync_;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropping dual Mid360 pair: stamp diff %.3fms exceeds tolerance %.3fms "
      "(drop_sync=%lu)",
      static_cast<double>(abs_stamp_diff_ns) / static_cast<double>(kNanosecondsPerMillisecond),
      sync_tolerance_ms_, drop_sync_);
    return;
  }

  const rclcpp::Time offset_base_stamp =
    primary_stamp <= secondary_stamp ? primary_stamp : secondary_stamp;
  rclcpp::Time published_stamp = offset_base_stamp;
  if (has_last_published_stamp_ && published_stamp <= last_published_stamp_) {
    published_stamp = last_published_stamp_ + rclcpp::Duration(0, 1);
    ++monotonic_nudge_count_;
  }

  const uint64_t primary_offset_rebase_ns =
    absoluteNanoseconds(stampDiffNanoseconds(primary_stamp, offset_base_stamp));
  const uint64_t secondary_offset_rebase_ns =
    absoluteNanoseconds(stampDiffNanoseconds(secondary_stamp, offset_base_stamp));

  CustomMsg output_msg;
  output_msg.header = primary_msg->header;
  output_msg.header.frame_id = common_frame_;
  output_msg.header.stamp = published_stamp;
  output_msg.timebase = 0;
  output_msg.lidar_id = primary_msg->lidar_id;
  output_msg.rsvd = primary_msg->rsvd;
  const size_t reserve_count =
    static_cast<size_t>(primary_msg->point_num) + static_cast<size_t>(secondary_msg->point_num);
  output_msg.points.reserve(reserve_count);

  for (const auto & point : primary_msg->points) {
    if (!isBeyondMinDistance(point, min_dist_primary_m_)) {
      ++drop_primary_min_dist_;
      continue;
    }
    appendPointWithRebasedOffset(
      point, primary_offset_rebase_ns, output_msg, drop_primary_offset_overflow_);
  }

  for (const auto & point : secondary_msg->points) {
    if (!isBeyondMinDistance(point, min_dist_secondary_m_)) {
      ++drop_secondary_min_dist_;
      continue;
    }

    const Eigen::Vector3d transformed_point =
      secondary_to_common_transform_ * Eigen::Vector3d(point.x, point.y, point.z);
    CustomPoint output_point;
    output_point.x = static_cast<float>(transformed_point.x());
    output_point.y = static_cast<float>(transformed_point.y());
    output_point.z = static_cast<float>(transformed_point.z());
    output_point.reflectivity = point.reflectivity;
    output_point.tag = point.tag;
    output_point.line = point.line;
    output_point.offset_time = point.offset_time;

    appendPointWithRebasedOffset(
      output_point, secondary_offset_rebase_ns, output_msg, drop_secondary_offset_overflow_);
  }

  std::stable_sort(
    output_msg.points.begin(), output_msg.points.end(),
    [](const CustomPoint & lhs, const CustomPoint & rhs) {
      return lhs.offset_time < rhs.offset_time;
    });
  output_msg.point_num = static_cast<uint32_t>(output_msg.points.size());
  const uint32_t published_point_num = output_msg.point_num;

  // Optional PC2 mirror for rviz. Build before moving output_msg away.
  // sensor_msgs/PointCloud2 layout: x y z (float32) + intensity (float32).
  // We map Livox 'reflectivity' (uint8) to intensity for a useful color map.
  if (publish_pc2_preview_ && merged_pc2_pub_) {
    auto pc2 = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pc2->header = output_msg.header;
    pc2->height = 1;
    pc2->width = published_point_num;
    pc2->is_dense = true;
    pc2->is_bigendian = false;
    pc2->fields.resize(4);
    const char * names[4] = {"x", "y", "z", "intensity"};
    for (size_t i = 0; i < 4; ++i) {
      pc2->fields[i].name = names[i];
      pc2->fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      pc2->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      pc2->fields[i].count = 1;
    }
    pc2->point_step = 4 * sizeof(float);
    pc2->row_step = pc2->point_step * pc2->width;
    pc2->data.resize(pc2->row_step);
    float * out = reinterpret_cast<float *>(pc2->data.data());
    for (const auto & pt : output_msg.points) {
      *out++ = pt.x;
      *out++ = pt.y;
      *out++ = pt.z;
      *out++ = static_cast<float>(pt.reflectivity);
    }
    merged_pc2_pub_->publish(std::move(pc2));
  }

  merged_pub_->publish(std::move(output_msg));
  last_published_stamp_ = published_stamp;
  has_last_published_stamp_ = true;
  ++msg_count_;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Dual Mid360 merger metrics: published=%lu last_points=%u drop_tf=%lu drop_sync=%lu "
    "drop_min_dist(primary/secondary)=%lu/%lu "
    "drop_offset_overflow(primary/secondary)=%lu/%lu "
    "stamp_nudges=%lu last_stamp_diff=%.3fms",
    msg_count_, published_point_num, drop_tf_, drop_sync_,
    drop_primary_min_dist_, drop_secondary_min_dist_, drop_primary_offset_overflow_,
    drop_secondary_offset_overflow_, monotonic_nudge_count_,
    static_cast<double>(abs_stamp_diff_ns) / static_cast<double>(kNanosecondsPerMillisecond));
}

bool MergerNode::ensureSecondaryToCommonTransform()
{
  if (has_secondary_to_common_transform_) {
    return true;
  }

  try {
    const geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(common_frame_, secondary_frame_, tf2::TimePointZero);
    secondary_to_common_transform_ = tf2::transformToEigen(transform_stamped.transform);
    has_secondary_to_common_transform_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Cached dual Mid360 transform %s -> %s",
      secondary_frame_.c_str(), common_frame_.c_str());
    return true;
  } catch (const tf2::TransformException & ex) {
    ++drop_tf_;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for dual Mid360 transform %s -> %s: %s (drop_tf=%lu)",
      secondary_frame_.c_str(), common_frame_.c_str(), ex.what(), drop_tf_);
    return false;
  }
}

bool MergerNode::appendPointWithRebasedOffset(
  const CustomPoint & point, uint64_t rebase_offset_ns, CustomMsg & output_msg,
  uint64_t & drop_overflow_count) const
{
  const uint64_t rebased_offset = static_cast<uint64_t>(point.offset_time) + rebase_offset_ns;
  if (rebased_offset > std::numeric_limits<uint32_t>::max()) {
    ++drop_overflow_count;
    return false;
  }

  CustomPoint output_point = point;
  output_point.offset_time = static_cast<uint32_t>(rebased_offset);
  output_msg.points.push_back(output_point);
  return true;
}

bool MergerNode::isBeyondMinDistance(const CustomPoint & point, double min_distance_m)
{
  const double dist_sq = static_cast<double>(point.x) * static_cast<double>(point.x) +
    static_cast<double>(point.y) * static_cast<double>(point.y) +
    static_cast<double>(point.z) * static_cast<double>(point.z);
  return dist_sq >= min_distance_m * min_distance_m;
}

}  // namespace sentry_dual_mid360

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_dual_mid360::MergerNode>());
  rclcpp::shutdown();
  return 0;
}
