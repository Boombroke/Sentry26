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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <cmath>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  accumulated_count_(0),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity()),
  has_localized_(false)
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});

  this->declare_parameter("max_iterations", 20);
  this->declare_parameter("accumulated_count_threshold", 20);
  this->declare_parameter("min_range", 0.5);
  this->declare_parameter("min_inlier_ratio", 0.3);
  this->declare_parameter("max_fitness_error", 1.0);
  this->declare_parameter("enable_periodic_relocalization", false);
  this->declare_parameter("relocalization_interval", 30.0);
  this->declare_parameter("max_correction_distance", 5.0);
  this->declare_parameter("emergency_max_dist_sq", 50.0);
  this->declare_parameter("emergency_consecutive_failures", 3);
  this->declare_parameter("terrain_clearing_threshold", 0.1);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("accumulated_count_threshold", accumulated_count_threshold_);
  this->get_parameter("min_range", min_range_);
  this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
  this->get_parameter("max_fitness_error", max_fitness_error_);
  this->get_parameter("enable_periodic_relocalization", enable_periodic_relocalization_);
  this->get_parameter("relocalization_interval", relocalization_interval_);
  this->get_parameter("max_correction_distance", max_correction_distance_);
  this->get_parameter("emergency_max_dist_sq", emergency_max_dist_sq_);
  this->get_parameter("emergency_consecutive_failures", emergency_consecutive_failures_);
  this->get_parameter("terrain_clearing_threshold", terrain_clearing_threshold_);

  RCLCPP_INFO(
    this->get_logger(),
    "Parameters: max_iterations=%d, accumulated_threshold=%d, min_range=%.2f, "
    "min_inlier_ratio=%.2f, max_fitness_error=%.2f, periodic=%s, interval=%.1fs, "
    "max_correction=%.1fm, emergency_max_dist_sq=%.1f, emergency_after=%d failures",
    max_iterations_, accumulated_count_threshold_, min_range_, min_inlier_ratio_,
    max_fitness_error_, enable_periodic_relocalization_ ? "true" : "false",
    relocalization_interval_, max_correction_distance_, emergency_max_dist_sq_,
    emergency_consecutive_failures_);

  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;
  accumulation_snapshot_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadPcdFile(prior_pcd_file_);

  map_clearing_pub_ = this->create_publisher<std_msgs::msg::Float32>("map_clearing", 1);
  cloud_clearing_pub_ = this->create_publisher<std_msgs::msg::Float32>("cloud_clearing", 1);

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  odom_to_lidar_odom_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
    "odom_to_lidar_odom", latched_qos,
    std::bind(
      &SmallGicpRelocalizationNode::odomToLidarOdomCallback, this, std::placeholders::_1));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadPcdFile(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());
}

void SmallGicpRelocalizationNode::odomToLidarOdomCallback(
  const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  if (global_map_ready_) {
    return;
  }
  Eigen::Affine3d odom_to_lidar_odom = tf2::transformToEigen(msg->transform);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "Received odom_to_lidar_odom from odom_bridge: translation = "
                          << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                          << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
  prepareTargetMap(odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::prepareTargetMap(const Eigen::Affine3d & odom_to_lidar_odom)
{
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  for (auto & pt : target_->points) {
    pt.z = 0.0f;
  }

  RCLCPP_INFO(
    this->get_logger(), "Target map after downsampling: %zu points (leaf_size=%.3f)",
    target_->size(), global_leaf_size_);

  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  global_map_ready_ = true;
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  if (!global_map_ready_) {
    return;
  }

  if (has_localized_ && !enable_periodic_relocalization_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filtered->reserve(scan->size());
  const double min_range_sq = min_range_ * min_range_;
  for (const auto & pt : scan->points) {
    double dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if (dist_sq >= min_range_sq) {
      filtered->push_back(pt);
    }
  }

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_count_ >= accumulated_count_threshold_) {
      accumulated_cloud_->clear();
      accumulated_count_ = 0;
    }
    *accumulated_cloud_ += *filtered;
    accumulated_count_++;
  }

  if (!has_localized_ && accumulated_count_ >= accumulated_count_threshold_) {
    RCLCPP_INFO(
      this->get_logger(), "Accumulated %d frames (%zu points), performing initial registration...",
      accumulated_count_, accumulated_cloud_->size());

    bool success = performRegistration(false);
    if (success) {
      has_localized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial localization succeeded.");

      if (enable_periodic_relocalization_) {
        periodic_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(relocalization_interval_ * 1000.0)),
          std::bind(&SmallGicpRelocalizationNode::periodicRegistrationCallback, this));
        RCLCPP_INFO(
          this->get_logger(), "Periodic relocalization enabled at %.1fs interval.",
          relocalization_interval_);
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Initial registration failed quality check. Will retry with more frames...");
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    accumulated_cloud_->clear();
    accumulated_count_ = 0;
  }
}

void SmallGicpRelocalizationNode::periodicRegistrationCallback()
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);

  if (accumulated_cloud_->empty() || accumulated_count_ < accumulated_count_threshold_ / 2) {
    RCLCPP_DEBUG(
      this->get_logger(), "Periodic reloc: insufficient points (%d frames, %zu points). Skipping.",
      accumulated_count_, accumulated_cloud_->size());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Periodic relocalization: %d frames (%zu points)",
    accumulated_count_, accumulated_cloud_->size());

  bool success = performRegistration(true);

  if (!success) {
    consecutive_periodic_failures_++;
    RCLCPP_WARN(
      this->get_logger(),
      "Periodic relocalization failed (%d/%d consecutive failures)",
      consecutive_periodic_failures_, emergency_consecutive_failures_);

    if (consecutive_periodic_failures_ >= emergency_consecutive_failures_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "EMERGENCY: %d consecutive failures detected — possible odometry divergence. "
        "Attempting emergency relocalization with expanded search...",
        consecutive_periodic_failures_);

      bool emergency_success = performEmergencyRegistration();
      if (emergency_success) {
        RCLCPP_WARN(this->get_logger(), "Emergency relocalization SUCCEEDED. Odometry corrected.");
        consecutive_periodic_failures_ = 0;
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Emergency relocalization FAILED. Robot may need manual intervention (2D Pose Estimate).");
      }
    }
  } else {
    consecutive_periodic_failures_ = 0;
  }

  accumulated_cloud_->clear();
  accumulated_count_ = 0;
  accumulation_snapshot_t_ = result_t_;
}

bool SmallGicpRelocalizationNode::performEmergencyRegistration()
{
  if (accumulated_cloud_->empty()) {
    return false;
  }

  auto source_cloud = accumulated_cloud_;

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *source_cloud, registered_leaf_size_ * 2.0);

  for (auto & pt : source_->points) {
    pt.z = 0.0f;
  }

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  RCLCPP_WARN(
    this->get_logger(),
    "Emergency GICP: source=%zu points, max_dist_sq=%.1f (normal=%.1f), max_iter=%d",
    source_->size(), emergency_max_dist_sq_, max_dist_sq_, max_iterations_ * 2);

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = emergency_max_dist_sq_;
  register_->optimizer.max_iterations = max_iterations_ * 2;

  // Multi-seed: last accepted result + 4 yaw perturbations (±45°, ±90°)
  struct Candidate {
    Eigen::Isometry3d guess;
    small_gicp::RegistrationResult result;
    bool valid = false;
    double score = 0.0;
  };

  std::vector<Candidate> candidates;
  Eigen::Isometry3d base_guess = previous_result_t_;
  double base_yaw = std::atan2(base_guess.rotation()(1, 0), base_guess.rotation()(0, 0));

  for (double yaw_offset : {0.0, M_PI / 4, -M_PI / 4, M_PI / 2, -M_PI / 2}) {
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.translation() = base_guess.translation();
    double yaw = base_yaw + yaw_offset;
    guess.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    candidates.push_back({guess, {}, false, 0.0});
  }

  int best_idx = -1;
  double best_score = -1.0;
  constexpr size_t kMinAbsoluteInliers = 50;

  for (size_t i = 0; i < candidates.size(); i++) {
    auto res = register_->align(*target_, *source_, *target_tree_, candidates[i].guess);
    candidates[i].result = res;

    if (!res.converged || res.num_inliers < kMinAbsoluteInliers) {
      continue;
    }

    double inlier_ratio = static_cast<double>(res.num_inliers) / source_->size();
    double fitness_error = res.error / static_cast<double>(res.num_inliers);

    if (inlier_ratio >= min_inlier_ratio_ * 0.5 &&
        fitness_error <= max_fitness_error_ * 2.0)
    {
      double score = static_cast<double>(res.num_inliers) / (fitness_error + 0.001);
      candidates[i].valid = true;
      candidates[i].score = score;

      if (score > best_score) {
        best_score = score;
        best_idx = static_cast<int>(i);
      }
    }
  }

  RCLCPP_WARN(
    this->get_logger(), "Emergency: tested %zu candidates, best_idx=%d, best_score=%.1f",
    candidates.size(), best_idx, best_score);

  bool accepted = (best_idx >= 0);
  auto result = accepted ? candidates[best_idx].result : candidates[0].result;

  if (!accepted) {
    return false;
  }

  const Eigen::Vector3d raw_t = result.T_target_source.translation();
  const Eigen::Matrix3d raw_r = result.T_target_source.rotation();
  double yaw = std::atan2(raw_r(1, 0), raw_r(0, 0));

  Eigen::Isometry3d constrained = Eigen::Isometry3d::Identity();
  constrained.translation() << raw_t.x(), raw_t.y(), 0.0;
  constrained.linear() =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  RCLCPP_WARN(
    this->get_logger(),
    "Emergency accepted: t=[%.3f, %.3f], yaw=%.3f (correction=%.3f m)",
    raw_t.x(), raw_t.y(), yaw,
    (constrained.translation() - result_t_.translation()).norm());

  result_t_ = previous_result_t_ = constrained;
  notifyTerrainClearing();
  return true;
}

bool SmallGicpRelocalizationNode::performRegistration(bool is_periodic)
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return false;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  for (auto & pt : source_->points) {
    pt.z = 0.0f;
  }

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "GICP input: source=%zu points, target=%zu points",
    source_->size(), target_->size());

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  const Eigen::Vector3d t = result.T_target_source.translation();
  const Eigen::Vector3d rpy =
    result.T_target_source.rotation().eulerAngles(0, 1, 2);

  RCLCPP_INFO(
    this->get_logger(),
    "GICP result: converged=%d, iterations=%zu, num_inliers=%zu, error=%.6f",
    result.converged, result.iterations, result.num_inliers, result.error);
  RCLCPP_INFO(
    this->get_logger(),
    "GICP transform: t=[%.3f, %.3f, %.3f], rpy=[%.3f, %.3f, %.3f]",
    t.x(), t.y(), t.z(), rpy.x(), rpy.y(), rpy.z());

  if (!result.converged) {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
    return false;
  }

  double inlier_ratio = static_cast<double>(result.num_inliers) / source_->size();
  RCLCPP_INFO(this->get_logger(), "GICP inlier_ratio=%.3f (threshold=%.3f)",
    inlier_ratio, min_inlier_ratio_);

  if (inlier_ratio < min_inlier_ratio_) {
    RCLCPP_WARN(
      this->get_logger(),
      "GICP quality check FAILED: inlier_ratio=%.3f < min_inlier_ratio=%.3f",
      inlier_ratio, min_inlier_ratio_);
    return false;
  }

  if (result.num_inliers > 0) {
    double fitness_error = result.error / static_cast<double>(result.num_inliers);
    RCLCPP_INFO(this->get_logger(), "GICP fitness_error=%.6f (threshold=%.6f)",
      fitness_error, max_fitness_error_);

    if (fitness_error > max_fitness_error_) {
      RCLCPP_WARN(
        this->get_logger(),
        "GICP quality check FAILED: fitness_error=%.6f > max_fitness_error=%.6f",
        fitness_error, max_fitness_error_);
      return false;
    }
  }

  if (is_periodic) {
    Eigen::Vector3d delta_t =
      result.T_target_source.translation() - result_t_.translation();
    double delta_dist = delta_t.norm();
    if (delta_dist > max_correction_distance_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Periodic reloc: correction too large (%.3f m > %.3f m). Rejecting.",
        delta_dist, max_correction_distance_);
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "Periodic reloc: accepted correction of %.3f m", delta_dist);
  }

  const Eigen::Vector3d raw_t = result.T_target_source.translation();
  const Eigen::Matrix3d raw_r = result.T_target_source.rotation();
  double yaw = std::atan2(raw_r(1, 0), raw_r(0, 0));

  Eigen::Isometry3d constrained = Eigen::Isometry3d::Identity();
  constrained.translation() << raw_t.x(), raw_t.y(), 0.0;
  constrained.linear() =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  RCLCPP_INFO(
    this->get_logger(),
    "Accepted 2D-constrained result: t=[%.3f, %.3f], yaw=%.3f",
    raw_t.x(), raw_t.y(), yaw);

  double correction_dist = (constrained.translation() - result_t_.translation()).norm();
  result_t_ = previous_result_t_ = constrained;

  if (correction_dist > terrain_clearing_threshold_) {
    notifyTerrainClearing();
    RCLCPP_WARN(
      this->get_logger(),
      "Correction %.3fm > threshold %.3fm, triggered terrain clearing.",
      correction_dist, terrain_clearing_threshold_);
  }

  return true;
}

void SmallGicpRelocalizationNode::notifyTerrainClearing()
{
  std_msgs::msg::Float32 msg;
  msg.data = 100.0f;
  map_clearing_pub_->publish(msg);
  cloud_clearing_pub_->publish(msg);
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Matrix3d rotation = result_t_.rotation();
  double yaw = std::atan2(rotation(1, 0), rotation(0, 0));

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = 0.0;

  transform_stamped.transform.rotation.x = 0.0;
  transform_stamped.transform.rotation.y = 0.0;
  transform_stamped.transform.rotation.z = std::sin(yaw / 2.0);
  transform_stamped.transform.rotation.w = std::cos(yaw / 2.0);

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    consecutive_periodic_failures_ = 0;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
