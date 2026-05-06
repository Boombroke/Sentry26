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
  , min_dist_front_m_(0.4)
  , min_dist_back_m_(0.4)
  , queue_size_(10)
  , tf_cache_retry_count_(10)
  , tf_cache_retry_interval_ms_(500)
  , has_back_to_common_transform_(false)
  , back_to_common_transform_(Eigen::Isometry3d::Identity())
  , has_last_published_stamp_(false)
  , msg_count_(0)
  , drop_tf_(0)
  , drop_sync_(0)
  , drop_front_min_dist_(0)
  , drop_back_min_dist_(0)
  , drop_front_offset_overflow_(0)
  , drop_back_offset_overflow_(0)
  , monotonic_nudge_count_(0)
{
  front_topic_ = this->declare_parameter<std::string>("front_topic", "/livox/lidar_front");
  back_topic_ = this->declare_parameter<std::string>("back_topic", "/livox/lidar_back");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/lidar");
  common_frame_ = this->declare_parameter<std::string>("common_frame", "front_mid360");
  back_frame_ = this->declare_parameter<std::string>("back_frame", "back_mid360");
  sync_tolerance_ms_ = this->declare_parameter<double>("sync_tolerance_ms", sync_tolerance_ms_);
  min_dist_front_m_ = this->declare_parameter<double>("min_dist_front_m", min_dist_front_m_);
  min_dist_back_m_ = this->declare_parameter<double>("min_dist_back_m", min_dist_back_m_);
  queue_size_ = this->declare_parameter<int>("queue_size", queue_size_);
  // Declared for YAML visibility; T8 retries TF every sync callback (no cadence use yet).
  tf_cache_retry_count_ =
    this->declare_parameter<int>("tf_cache_retry_count", tf_cache_retry_count_);
  tf_cache_retry_interval_ms_ =
    this->declare_parameter<int>("tf_cache_retry_interval_ms", tf_cache_retry_interval_ms_);
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

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.depth = static_cast<size_t>(queue_size_);
  front_sub_.subscribe(this, front_topic_, qos_profile);
  back_sub_.subscribe(this, back_topic_, qos_profile);

  SyncPolicy sync_policy(static_cast<uint32_t>(queue_size_));
  sync_policy.setMaxIntervalDuration(
    rclcpp::Duration::from_seconds(sync_tolerance_ms_ / 1000.0));
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    static_cast<const SyncPolicy &>(sync_policy), front_sub_, back_sub_);
  sync_->registerCallback(std::bind(
    &MergerNode::mergeCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    this->get_logger(),
    "sentry_dual_mid360 merger_node started: front=%s back=%s output=%s common_frame=%s "
    "back_frame=%s tolerance=%.3fms queue_size=%d",
    front_topic_.c_str(), back_topic_.c_str(), output_topic_.c_str(), common_frame_.c_str(),
    back_frame_.c_str(), sync_tolerance_ms_, queue_size_);
}

void MergerNode::mergeCallback(
  const CustomMsg::ConstSharedPtr & front_msg, const CustomMsg::ConstSharedPtr & back_msg)
{
  if (!ensureBackToCommonTransform()) {
    return;
  }

  const rclcpp::Time front_stamp(front_msg->header.stamp);
  const rclcpp::Time back_stamp(back_msg->header.stamp);
  const int64_t signed_stamp_diff_ns = stampDiffNanoseconds(front_stamp, back_stamp);
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

  const rclcpp::Time offset_base_stamp = front_stamp <= back_stamp ? front_stamp : back_stamp;
  rclcpp::Time published_stamp = offset_base_stamp;
  if (has_last_published_stamp_ && published_stamp <= last_published_stamp_) {
    published_stamp = last_published_stamp_ + rclcpp::Duration(0, 1);
    ++monotonic_nudge_count_;
  }

  const uint64_t front_offset_rebase_ns =
    absoluteNanoseconds(stampDiffNanoseconds(front_stamp, offset_base_stamp));
  const uint64_t back_offset_rebase_ns =
    absoluteNanoseconds(stampDiffNanoseconds(back_stamp, offset_base_stamp));

  CustomMsg output_msg;
  output_msg.header = front_msg->header;
  output_msg.header.frame_id = common_frame_;
  output_msg.header.stamp = published_stamp;
  output_msg.timebase = 0;
  output_msg.lidar_id = front_msg->lidar_id;
  output_msg.rsvd = front_msg->rsvd;
  const size_t reserve_count =
    static_cast<size_t>(front_msg->point_num) + static_cast<size_t>(back_msg->point_num);
  output_msg.points.reserve(reserve_count);

  for (const auto & point : front_msg->points) {
    if (!isBeyondMinDistance(point, min_dist_front_m_)) {
      ++drop_front_min_dist_;
      continue;
    }
    appendPointWithRebasedOffset(
      point, front_offset_rebase_ns, output_msg, drop_front_offset_overflow_);
  }

  for (const auto & point : back_msg->points) {
    if (!isBeyondMinDistance(point, min_dist_back_m_)) {
      ++drop_back_min_dist_;
      continue;
    }

    const Eigen::Vector3d transformed_point =
      back_to_common_transform_ * Eigen::Vector3d(point.x, point.y, point.z);
    CustomPoint output_point;
    output_point.x = static_cast<float>(transformed_point.x());
    output_point.y = static_cast<float>(transformed_point.y());
    output_point.z = static_cast<float>(transformed_point.z());
    output_point.reflectivity = point.reflectivity;
    output_point.tag = point.tag;
    output_point.line = point.line;
    output_point.offset_time = point.offset_time;

    appendPointWithRebasedOffset(
      output_point, back_offset_rebase_ns, output_msg, drop_back_offset_overflow_);
  }

  std::stable_sort(
    output_msg.points.begin(), output_msg.points.end(),
    [](const CustomPoint & lhs, const CustomPoint & rhs) {
      return lhs.offset_time < rhs.offset_time;
    });
  output_msg.point_num = static_cast<uint32_t>(output_msg.points.size());
  const uint32_t published_point_num = output_msg.point_num;

  merged_pub_->publish(std::move(output_msg));
  last_published_stamp_ = published_stamp;
  has_last_published_stamp_ = true;
  ++msg_count_;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Dual Mid360 merger metrics: published=%lu last_points=%u drop_tf=%lu drop_sync=%lu "
    "drop_min_dist(front/back)=%lu/%lu drop_offset_overflow(front/back)=%lu/%lu "
    "stamp_nudges=%lu last_stamp_diff=%.3fms",
    msg_count_, published_point_num, drop_tf_, drop_sync_,
    drop_front_min_dist_, drop_back_min_dist_, drop_front_offset_overflow_,
    drop_back_offset_overflow_, monotonic_nudge_count_,
    static_cast<double>(abs_stamp_diff_ns) / static_cast<double>(kNanosecondsPerMillisecond));
}

bool MergerNode::ensureBackToCommonTransform()
{
  if (has_back_to_common_transform_) {
    return true;
  }

  try {
    const geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_->lookupTransform(common_frame_, back_frame_, tf2::TimePointZero);
    back_to_common_transform_ = tf2::transformToEigen(transform_stamped.transform);
    has_back_to_common_transform_ = true;
    RCLCPP_INFO(
      this->get_logger(), "Cached dual Mid360 transform %s -> %s", back_frame_.c_str(),
      common_frame_.c_str());
    return true;
  } catch (const tf2::TransformException & ex) {
    ++drop_tf_;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for dual Mid360 transform %s -> %s: %s (drop_tf=%lu)", back_frame_.c_str(),
      common_frame_.c_str(), ex.what(), drop_tf_);
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
