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

#include <chrono>
#include <cmath>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
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
  this->declare_parameter("max_fitness_error", 0.05);
  this->declare_parameter("enable_periodic_relocalization", false);
  this->declare_parameter("relocalization_interval", 30.0);
  this->declare_parameter("max_correction_distance", 5.0);
  this->declare_parameter("emergency_max_dist_sq", 50.0);
  this->declare_parameter("emergency_consecutive_failures", 3);
  this->declare_parameter("emergency_max_correction_distance", 3.0);
  this->declare_parameter("emergency_min_score_threshold", 200000.0);
  this->declare_parameter("quality_convergence_threshold", 0.008);
  this->declare_parameter("terrain_clearing_threshold", 0.1);

  this->declare_parameter("enable_deep_verification", false);
  this->declare_parameter("deep_verification_interval", 30.0);
  this->declare_parameter("deep_accumulated_count_threshold", 100);
  this->declare_parameter("deep_max_dist_sq", 225.0);
  this->declare_parameter("deep_max_iterations", 300);
  this->declare_parameter("deep_global_leaf_size", 0.05);
  this->declare_parameter("deep_registered_leaf_size", 0.05);
  this->declare_parameter("deep_min_inlier_ratio", 0.3);
  this->declare_parameter("deep_max_fitness_error", 0.5);
  this->declare_parameter("deep_max_correction_distance", 15.0);
  this->declare_parameter("deep_min_score_threshold", 500000.0);

  this->declare_parameter("health_window_size", 5);
  this->declare_parameter("health_unhealthy_score_threshold", 50000.0);
  this->declare_parameter("health_stale_timeout_sec", 30.0);

  this->declare_parameter("enable_global_relocalization", false);
  this->declare_parameter("scan_context_db_file", "");
  this->declare_parameter("sc_num_rings", 20);
  this->declare_parameter("sc_num_sectors", 60);
  this->declare_parameter("sc_max_radius", 30.0);
  this->declare_parameter("sc_height_min", -2.0);
  this->declare_parameter("global_top_k", 5);
  this->declare_parameter("global_relocalization_min_interval_sec", 30.0);
  this->declare_parameter("global_max_correction_distance", 30.0);
  this->declare_parameter("global_min_score_threshold", 300000.0);
  this->declare_parameter("global_accumulated_count_threshold", 50);
  this->declare_parameter("global_max_dist_sq", 225.0);
  this->declare_parameter("global_max_iterations", 200);
  this->declare_parameter("global_registered_leaf_size", 0.05);

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
  this->get_parameter("emergency_max_correction_distance", emergency_max_correction_distance_);
  this->get_parameter("emergency_min_score_threshold", emergency_min_score_threshold_);
  this->get_parameter("quality_convergence_threshold", quality_convergence_threshold_);
  this->get_parameter("terrain_clearing_threshold", terrain_clearing_threshold_);

  this->get_parameter("enable_deep_verification", enable_deep_verification_);
  this->get_parameter("deep_verification_interval", deep_verification_interval_);
  this->get_parameter("deep_accumulated_count_threshold", deep_accumulated_count_threshold_);
  this->get_parameter("deep_max_dist_sq", deep_max_dist_sq_);
  this->get_parameter("deep_max_iterations", deep_max_iterations_);
  this->get_parameter("deep_global_leaf_size", deep_global_leaf_size_);
  this->get_parameter("deep_registered_leaf_size", deep_registered_leaf_size_);
  this->get_parameter("deep_min_inlier_ratio", deep_min_inlier_ratio_);
  this->get_parameter("deep_max_fitness_error", deep_max_fitness_error_);
  this->get_parameter("deep_max_correction_distance", deep_max_correction_distance_);
  this->get_parameter("deep_min_score_threshold", deep_min_score_threshold_);

  {
    int health_window_size_param = static_cast<int>(health_window_size_);
    this->get_parameter("health_window_size", health_window_size_param);
    if (health_window_size_param < 1) {
      health_window_size_param = 1;
    }
    health_window_size_ = static_cast<size_t>(health_window_size_param);
  }
  this->get_parameter("health_unhealthy_score_threshold", health_unhealthy_score_threshold_);
  this->get_parameter("health_stale_timeout_sec", health_stale_timeout_sec_);

  this->get_parameter("enable_global_relocalization", enable_global_relocalization_);
  this->get_parameter("scan_context_db_file", scan_context_db_file_);
  {
    int sc_num_rings = sc_config_.num_rings;
    int sc_num_sectors = sc_config_.num_sectors;
    double sc_max_radius = sc_config_.max_radius;
    double sc_height_min = sc_config_.height_min;
    this->get_parameter("sc_num_rings", sc_num_rings);
    this->get_parameter("sc_num_sectors", sc_num_sectors);
    this->get_parameter("sc_max_radius", sc_max_radius);
    this->get_parameter("sc_height_min", sc_height_min);
    sc_config_.num_rings = sc_num_rings;
    sc_config_.num_sectors = sc_num_sectors;
    sc_config_.max_radius = sc_max_radius;
    sc_config_.height_min = sc_height_min;
  }
  this->get_parameter("global_top_k", global_top_k_);
  this->get_parameter(
    "global_relocalization_min_interval_sec", global_relocalization_min_interval_sec_);
  this->get_parameter("global_max_correction_distance", global_max_correction_distance_);
  this->get_parameter("global_min_score_threshold", global_min_score_threshold_);
  this->get_parameter("global_accumulated_count_threshold", global_accumulated_count_threshold_);
  this->get_parameter("global_max_dist_sq", global_max_dist_sq_);
  this->get_parameter("global_max_iterations", global_max_iterations_);
  {
    double leaf = static_cast<double>(global_registered_leaf_size_);
    this->get_parameter("global_registered_leaf_size", leaf);
    global_registered_leaf_size_ = static_cast<float>(leaf);
  }
  sc_engine_ = std::make_unique<ScanContextEngine>(sc_config_);

  HealthConfig health_cfg;
  health_cfg.window_size = health_window_size_;
  health_cfg.unhealthy_score_threshold = health_unhealthy_score_threshold_;
  health_cfg.stale_timeout_sec = health_stale_timeout_sec_;
  health_monitor_ = std::make_unique<LocalizationHealth>(health_cfg);

  RCLCPP_INFO(
    this->get_logger(),
    "Health monitor: window=%zu, unhealthy_score_threshold=%.0f, stale_timeout=%.1fs",
    health_window_size_, health_unhealthy_score_threshold_, health_stale_timeout_sec_);

  RCLCPP_INFO(
    this->get_logger(),
    "Parameters: max_iterations=%d, accumulated_threshold=%d, min_range=%.2f, "
    "min_inlier_ratio=%.2f, max_fitness_error=%.2f, quality_convergence_threshold=%.4f, "
    "periodic=%s, interval=%.1fs, "
    "max_correction=%.1fm, emergency_max_dist_sq=%.1f, emergency_after=%d failures, "
    "emergency_max_correction=%.1fm, emergency_min_score=%.1f",
    max_iterations_, accumulated_count_threshold_, min_range_, min_inlier_ratio_,
    max_fitness_error_, quality_convergence_threshold_,
    enable_periodic_relocalization_ ? "true" : "false", relocalization_interval_,
    max_correction_distance_, emergency_max_dist_sq_, emergency_consecutive_failures_,
    emergency_max_correction_distance_, emergency_min_score_threshold_);

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
  deep_accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  deep_register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadPcdFile(prior_pcd_file_);

  if (enable_global_relocalization_) {
    prepareScanContextDB();
  }
  last_global_attempt_time_ = std::chrono::steady_clock::now();

  map_clearing_pub_ = this->create_publisher<std_msgs::msg::Float32>("map_clearing", 1);
  cloud_clearing_pub_ = this->create_publisher<std_msgs::msg::Float32>("cloud_clearing", 1);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&SmallGicpRelocalizationNode::publishTransform, this));

  if (enable_deep_verification_) {
    deep_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(deep_verification_interval_ * 1000.0)),
      std::bind(&SmallGicpRelocalizationNode::deepVerificationTimerCallback, this));
    RCLCPP_INFO(
      this->get_logger(),
      "Deep verification enabled: interval=%.1fs, leaf=%.3f, max_dist_sq=%.1f, "
      "max_iter=%d, accum=%d, min_score=%.0f, max_corr=%.1fm",
      deep_verification_interval_, deep_global_leaf_size_, deep_max_dist_sq_, deep_max_iterations_,
      deep_accumulated_count_threshold_, deep_min_score_threshold_, deep_max_correction_distance_);
  }
}

void SmallGicpRelocalizationNode::loadPcdFile(const std::string & file_name)
{
  if (file_name.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "No prior PCD file configured. Target map remains empty; GICP registration will be skipped.");
    return;
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());
  prepareTargetMap();
  if (enable_deep_verification_) {
    prepareDeepTargetMap();
  }
}

void SmallGicpRelocalizationNode::prepareTargetMap()
{
  if (!global_map_ || global_map_->empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Prior PCD map is empty. Target map remains empty; GICP registration will be skipped.");
    global_map_ready_ = false;
    return;
  }

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  if (!target_ || target_->empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Target map is empty after downsampling. GICP registration will be skipped.");
    global_map_ready_ = false;
    return;
  }

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

void SmallGicpRelocalizationNode::prepareDeepTargetMap()
{
  if (!global_map_ || global_map_->empty()) {
    return;
  }

  target_deep_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, deep_global_leaf_size_);

  if (!target_deep_ || target_deep_->empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Deep target map is empty after downsampling. Deep verification disabled.");
    enable_deep_verification_ = false;
    return;
  }

  for (auto & pt : target_deep_->points) {
    pt.z = 0.0f;
  }

  small_gicp::estimate_covariances_omp(*target_deep_, num_neighbors_, num_threads_);

  target_deep_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_deep_, small_gicp::KdTreeBuilderOMP(num_threads_));

  RCLCPP_INFO(
    this->get_logger(), "Deep target map ready: %zu points (leaf_size=%.3f)", target_deep_->size(),
    deep_global_leaf_size_);
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

  if (enable_deep_verification_) {
    std::lock_guard<std::mutex> lock(deep_cloud_mutex_);
    if (deep_accumulated_count_ >= deep_accumulated_count_threshold_) {
      deep_accumulated_cloud_->clear();
      deep_accumulated_count_ = 0;
    }
    *deep_accumulated_cloud_ += *filtered;
    deep_accumulated_count_++;
  }

  if (!has_localized_ && accumulated_count_ >= accumulated_count_threshold_) {
    RCLCPP_INFO(
      this->get_logger(), "Accumulated %d frames (%zu points), performing initial registration...",
      accumulated_count_, accumulated_cloud_->size());

    bool sc_success = false;

    // 冷启动优先走 Scan Context 全局重定位（如果 .scdb 已加载）。
    // SC 不依赖 init_pose / LIO 重力收敛 yaw，能在机器人离 PCD 建图原点 1m+ 时给出
    // 准确初值，避免 GICP 从 identity + 3m 关联半径起跑锁次优局部最小。
    if (global_relocalization_ready_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Cold start: trying Scan Context global relocalization first "
        "(does not depend on init_pose / LIO orientation)");
      Eigen::Vector3d robot_in_odom;
      bool tf_ok = false;
      try {
        auto tf_stamped = tf_buffer_->lookupTransform(
          odom_frame_, robot_base_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.5));
        robot_in_odom << tf_stamped.transform.translation.x, tf_stamped.transform.translation.y,
          tf_stamped.transform.translation.z;
        tf_ok = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          this->get_logger(),
          "Cold start SC: TF lookup %s->%s failed: %s. Falling back to GICP-only.",
          odom_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      }
      if (tf_ok) {
        // 深拷贝 snapshot，不影响后续 GICP 精调用 accumulated_cloud_ 本体
        pcl::PointCloud<pcl::PointXYZ>::Ptr snapshot;
        {
          std::lock_guard<std::mutex> lock(cloud_mutex_);
          snapshot = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*accumulated_cloud_);
        }
        // 同步调用（不 detach）：节点冷启动期间 publishTransform 50ms 一次发的
        // result_t_ 还是 identity，SC 跑慢点没关系；失败也只是降级到 GICP-only。
        sc_success = runGlobalRelocalization(snapshot, robot_in_odom);
        if (sc_success) {
          RCLCPP_WARN(
            this->get_logger(),
            "Cold start SC SUCCEEDED. Refining with GICP from SC initial guess.");
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "Cold start SC failed. Falling back to identity-init GICP.");
        }
      }
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Cold start: .scdb not loaded, using identity-init GICP "
        "(may fail if start position differs from mapping origin).");
    }

    // performRegistration 用 previous_result_t_ 当 GICP 初值：
    //   - SC 成功：previous_result_t_ 已是 SC 结果（runGlobalRelocalization 写好），
    //              GICP 在准确初值上 3m 半径精调，必收敛到厘米级
    //   - SC 失败：previous_result_t_ 仍是 init_pose（identity），GICP 自己起跑（旧行为）
    bool success = performRegistration(false);
    if (success) {
      has_localized_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Initial localization succeeded%s.",
        sc_success ? " (SC + GICP refine)" : " (GICP only)");

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
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    if (accumulated_cloud_->empty() || accumulated_count_ < accumulated_count_threshold_ / 2) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Periodic reloc: insufficient points (%d frames, %zu points). Skipping.",
        accumulated_count_, accumulated_cloud_->size());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Periodic relocalization: %d frames (%zu points)", accumulated_count_,
      accumulated_cloud_->size());

    bool success = performRegistration(true);

    if (!success) {
      consecutive_periodic_failures_++;
      RCLCPP_WARN(
        this->get_logger(), "Periodic relocalization failed (%d/%d consecutive failures)",
        consecutive_periodic_failures_, emergency_consecutive_failures_);

      if (consecutive_periodic_failures_ >= emergency_consecutive_failures_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "EMERGENCY: %d consecutive failures detected — possible odometry divergence. "
          "Attempting emergency relocalization with expanded search...",
          consecutive_periodic_failures_);

        bool emergency_success = performEmergencyRegistration();
        if (emergency_success) {
          RCLCPP_WARN(
            this->get_logger(), "Emergency relocalization SUCCEEDED. Odometry corrected.");
          consecutive_periodic_failures_ = 0;
        } else {
          RCLCPP_ERROR(
            this->get_logger(),
            "Emergency relocalization FAILED. "
            "Robot may need manual intervention (2D Pose Estimate).");
        }
      }
    } else {
      consecutive_periodic_failures_ = 0;
    }

    // NOTE: do NOT clear accumulated_cloud_ here anymore. The global relocalization layer
    // (checkAndTriggerGlobalRelocalization, called after releasing this lock) needs the
    // accumulated cloud to build the query Scan Context. The clear/reset is performed
    // inside the global trigger path, or in registeredPcdCallback when the next batch
    // exceeds accumulated_count_threshold_.
    accumulation_snapshot_t_ = result_t_;
  }

  if (health_monitor_) {
    auto m = health_monitor_->getMetrics();
    if (m.unhealthy) {
      RCLCPP_WARN(
        this->get_logger(),
        "Localization health UNHEALTHY: median_score=%.0f (threshold=%.0f), "
        "min=%.0f, samples=%zu, idle=%.1fs",
        m.median_score, health_monitor_->config().unhealthy_score_threshold, m.min_score,
        m.sample_count, m.seconds_since_last_success);
    } else if (m.sample_count >= health_monitor_->config().window_size) {
      RCLCPP_DEBUG(
        this->get_logger(), "Localization health OK: median_score=%.0f, samples=%zu",
        m.median_score, m.sample_count);
    }
  }

  checkAndTriggerGlobalRelocalization();

  // Reset accumulator after the global trigger had a chance to snapshot it.
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    accumulated_cloud_->clear();
    accumulated_count_ = 0;
  }
}

bool SmallGicpRelocalizationNode::performEmergencyRegistration()
{
  if (accumulated_cloud_->empty()) {
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
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
  struct Candidate
  {
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

    if (inlier_ratio >= min_inlier_ratio_ * 0.5 && fitness_error <= max_fitness_error_ * 2.0) {
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

  bool accepted = (best_idx >= 0) && (best_score >= emergency_min_score_threshold_);
  if (best_idx >= 0 && best_score < emergency_min_score_threshold_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Emergency REJECTED: best_score=%.1f below threshold=%.1f. "
      "Possible false match, waiting for next cycle.",
      best_score, emergency_min_score_threshold_);
  }
  auto result = accepted ? candidates[best_idx].result : candidates[0].result;

  if (!accepted) {
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return false;
  }

  const Eigen::Vector3d raw_t = result.T_target_source.translation();
  const Eigen::Matrix3d raw_r = result.T_target_source.rotation();
  double yaw = std::atan2(raw_r(1, 0), raw_r(0, 0));

  Eigen::Isometry3d constrained = Eigen::Isometry3d::Identity();
  constrained.translation() << raw_t.x(), raw_t.y(), 0.0;
  constrained.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  RCLCPP_WARN(
    this->get_logger(), "Emergency accepted: t=[%.3f, %.3f], yaw=%.3f (correction=%.3f m)",
    raw_t.x(), raw_t.y(), yaw, (constrained.translation() - result_t_.translation()).norm());

  // 硬限制：Emergency 修正距离不得超过 emergency_max_correction_distance_
  double emergency_correction = (constrained.translation() - result_t_.translation()).norm();
  if (emergency_correction > emergency_max_correction_distance_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Emergency REJECTED: correction %.3f m exceeds emergency_max_correction_distance %.3f m. "
      "Robot may need manual /initialpose.",
      emergency_correction, emergency_max_correction_distance_);
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return false;
  }

  result_t_ = previous_result_t_ = constrained;
  notifyTerrainClearing();
  if (health_monitor_) {
    health_monitor_->recordSuccess(best_score);
  }
  return true;
}

bool SmallGicpRelocalizationNode::performRegistration(bool is_periodic)
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
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
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "GICP input: source=%zu points, target=%zu points", source_->size(),
    target_->size());

  if (!target_ || target_->empty() || !target_tree_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Target map empty or not loaded (prior PCD missing). "
      "Skipping GICP registration. SLAM mode doesn't need PCD.");
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return false;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  const Eigen::Vector3d t = result.T_target_source.translation();
  const Eigen::Vector3d rpy = result.T_target_source.rotation().eulerAngles(0, 1, 2);

  RCLCPP_INFO(
    this->get_logger(), "GICP result: converged=%d, iterations=%zu, num_inliers=%zu, error=%.6f",
    result.converged, result.iterations, result.num_inliers, result.error);
  RCLCPP_INFO(
    this->get_logger(), "GICP transform: t=[%.3f, %.3f, %.3f], rpy=[%.3f, %.3f, %.3f]", t.x(),
    t.y(), t.z(), rpy.x(), rpy.y(), rpy.z());

  if (!result.converged && result.num_inliers > 0) {
    double per_point_err = result.error / static_cast<double>(result.num_inliers);
    if (per_point_err < quality_convergence_threshold_) {
      RCLCPP_INFO(
        this->get_logger(),
        "GICP did not formally converge (iterations=%zu) but per_point_error=%.6f < "
        "quality_convergence_threshold=%.6f, treating as quality-converged.",
        result.iterations, per_point_err, quality_convergence_threshold_);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "GICP did not converge: iterations=%zu, per_point_error=%.6f >= threshold=%.6f",
        result.iterations, per_point_err, quality_convergence_threshold_);
      if (health_monitor_) {
        health_monitor_->recordFailure();
      }
      return false;
    }
  } else if (!result.converged) {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge (no inliers).");
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return false;
  }

  double inlier_ratio = static_cast<double>(result.num_inliers) / source_->size();
  RCLCPP_INFO(
    this->get_logger(), "GICP inlier_ratio=%.3f (threshold=%.3f)", inlier_ratio, min_inlier_ratio_);

  if (inlier_ratio < min_inlier_ratio_) {
    RCLCPP_WARN(
      this->get_logger(), "GICP quality check FAILED: inlier_ratio=%.3f < min_inlier_ratio=%.3f",
      inlier_ratio, min_inlier_ratio_);
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return false;
  }

  double fitness_error = 0.0;
  if (result.num_inliers > 0) {
    fitness_error = result.error / static_cast<double>(result.num_inliers);
    RCLCPP_INFO(
      this->get_logger(), "GICP fitness_error=%.6f (threshold=%.6f)", fitness_error,
      max_fitness_error_);

    if (fitness_error > max_fitness_error_) {
      RCLCPP_WARN(
        this->get_logger(),
        "GICP quality check FAILED: fitness_error=%.6f > max_fitness_error=%.6f", fitness_error,
        max_fitness_error_);
      if (health_monitor_) {
        health_monitor_->recordFailure();
      }
      return false;
    }
  }

  if (is_periodic) {
    Eigen::Vector3d delta_t = result.T_target_source.translation() - result_t_.translation();
    double delta_dist = delta_t.norm();
    if (delta_dist > max_correction_distance_) {
      RCLCPP_WARN(
        this->get_logger(), "Periodic reloc: correction too large (%.3f m > %.3f m). Rejecting.",
        delta_dist, max_correction_distance_);
      if (health_monitor_) {
        health_monitor_->recordFailure();
      }
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Periodic reloc: accepted correction of %.3f m", delta_dist);
  }

  const Eigen::Vector3d raw_t = result.T_target_source.translation();
  const Eigen::Matrix3d raw_r = result.T_target_source.rotation();
  double yaw = std::atan2(raw_r(1, 0), raw_r(0, 0));

  Eigen::Isometry3d constrained = Eigen::Isometry3d::Identity();
  constrained.translation() << raw_t.x(), raw_t.y(), 0.0;
  constrained.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  RCLCPP_INFO(
    this->get_logger(), "Accepted 2D-constrained result: t=[%.3f, %.3f], yaw=%.3f", raw_t.x(),
    raw_t.y(), yaw);

  double correction_dist = (constrained.translation() - result_t_.translation()).norm();
  result_t_ = previous_result_t_ = constrained;

  if (correction_dist > terrain_clearing_threshold_) {
    notifyTerrainClearing();
    RCLCPP_WARN(
      this->get_logger(), "Correction %.3fm > threshold %.3fm, triggered terrain clearing.",
      correction_dist, terrain_clearing_threshold_);
  }

  if (health_monitor_ && result.num_inliers > 0) {
    double score = static_cast<double>(result.num_inliers) / (fitness_error + 0.001);
    health_monitor_->recordSuccess(score);
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
    has_localized_ = true;
    consecutive_periodic_failures_ = 0;

    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      accumulated_cloud_->clear();
      accumulated_count_ = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Initial pose accepted. Marked as localized.");
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

void SmallGicpRelocalizationNode::deepVerificationTimerCallback()
{
  if (!global_map_ready_ || !target_deep_ || !target_deep_tree_) {
    return;
  }
  if (!has_localized_) {
    return;
  }
  if (deep_running_.load()) {
    RCLCPP_DEBUG(
      this->get_logger(), "Deep verification still running from previous tick. Skipping.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr snapshot;
  {
    std::lock_guard<std::mutex> lock(deep_cloud_mutex_);
    if (
      deep_accumulated_cloud_->empty() ||
      deep_accumulated_count_ < deep_accumulated_count_threshold_ / 2) {
      RCLCPP_DEBUG(
        this->get_logger(), "Deep verification: insufficient frames (%d/%d). Skipping.",
        deep_accumulated_count_, deep_accumulated_count_threshold_);
      return;
    }
    snapshot = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*deep_accumulated_cloud_);
    deep_accumulated_cloud_->clear();
    deep_accumulated_count_ = 0;
  }

  Eigen::Isometry3d initial_guess = result_t_;

  deep_running_.store(true);
  std::thread([this, snapshot, initial_guess]() {
    runDeepVerification(snapshot, initial_guess);
    deep_running_.store(false);
  }).detach();
}

void SmallGicpRelocalizationNode::runDeepVerification(
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_snapshot, Eigen::Isometry3d initial_guess)
{
  if (!accumulated_snapshot || accumulated_snapshot->empty()) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Deep verification: %zu raw points, leaf=%.3f, max_dist_sq=%.1f, max_iter=%d",
    accumulated_snapshot->size(), deep_global_leaf_size_, deep_max_dist_sq_, deep_max_iterations_);

  auto deep_source = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_snapshot, deep_registered_leaf_size_);

  if (!deep_source || deep_source->empty()) {
    RCLCPP_WARN(this->get_logger(), "Deep verification: source empty after downsample.");
    return;
  }

  for (auto & pt : deep_source->points) {
    pt.z = 0.0f;
  }

  small_gicp::estimate_covariances_omp(*deep_source, num_neighbors_, num_threads_);
  auto deep_source_tree =
    std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
      deep_source, small_gicp::KdTreeBuilderOMP(num_threads_));

  deep_register_->reduction.num_threads = num_threads_;
  deep_register_->rejector.max_dist_sq = deep_max_dist_sq_;
  deep_register_->optimizer.max_iterations = deep_max_iterations_;

  auto t_start = std::chrono::steady_clock::now();
  auto result =
    deep_register_->align(*target_deep_, *deep_source, *target_deep_tree_, initial_guess);
  auto t_end = std::chrono::steady_clock::now();
  double elapsed_ms =
    std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count() / 1000.0;

  if (!result.converged && result.num_inliers > 0) {
    double per_point_err = result.error / static_cast<double>(result.num_inliers);
    if (per_point_err >= quality_convergence_threshold_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Deep verification did NOT converge (iter=%zu, per_point=%.6f). Skipping.",
        result.iterations, per_point_err);
      if (health_monitor_) {
        health_monitor_->recordFailure();
      }
      return;
    }
  } else if (!result.converged) {
    RCLCPP_WARN(this->get_logger(), "Deep verification: no inliers, skipping.");
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return;
  }

  if (result.num_inliers == 0) {
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return;
  }

  double inlier_ratio = static_cast<double>(result.num_inliers) / deep_source->size();
  double fitness_error = result.error / static_cast<double>(result.num_inliers);
  double score = static_cast<double>(result.num_inliers) / (fitness_error + 0.001);

  RCLCPP_INFO(
    this->get_logger(),
    "Deep verification done in %.0f ms: inliers=%zu (ratio=%.3f), fitness=%.6f, score=%.0f",
    elapsed_ms, result.num_inliers, inlier_ratio, fitness_error, score);

  if (inlier_ratio < deep_min_inlier_ratio_) {
    RCLCPP_WARN(
      this->get_logger(), "Deep verification REJECTED: inlier_ratio=%.3f < %.3f.", inlier_ratio,
      deep_min_inlier_ratio_);
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return;
  }
  if (fitness_error > deep_max_fitness_error_) {
    RCLCPP_WARN(
      this->get_logger(), "Deep verification REJECTED: fitness_error=%.6f > %.6f.", fitness_error,
      deep_max_fitness_error_);
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return;
  }
  if (score < deep_min_score_threshold_) {
    RCLCPP_WARN(
      this->get_logger(), "Deep verification REJECTED: score=%.0f < %.0f. Possible false match.",
      score, deep_min_score_threshold_);
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return;
  }

  const Eigen::Vector3d raw_t = result.T_target_source.translation();
  const Eigen::Matrix3d raw_r = result.T_target_source.rotation();
  double yaw = std::atan2(raw_r(1, 0), raw_r(0, 0));

  Eigen::Isometry3d constrained = Eigen::Isometry3d::Identity();
  constrained.translation() << raw_t.x(), raw_t.y(), 0.0;
  constrained.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  double correction_dist = (constrained.translation() - result_t_.translation()).norm();
  if (correction_dist > deep_max_correction_distance_) {
    RCLCPP_WARN(
      this->get_logger(), "Deep verification REJECTED: correction %.3fm > %.3fm.", correction_dist,
      deep_max_correction_distance_);
    if (health_monitor_) {
      health_monitor_->recordFailure();
    }
    return;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "Deep verification ACCEPTED: t=[%.3f, %.3f], yaw=%.3f, correction=%.3fm, score=%.0f", raw_t.x(),
    raw_t.y(), yaw, correction_dist, score);

  result_t_ = previous_result_t_ = constrained;

  if (correction_dist > terrain_clearing_threshold_) {
    notifyTerrainClearing();
  }

  if (health_monitor_) {
    health_monitor_->recordSuccess(score);
  }
}

void SmallGicpRelocalizationNode::prepareScanContextDB()
{
  if (scan_context_db_file_.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Global relocalization enabled but scan_context_db_file is empty. Disabling.");
    return;
  }
  sc_db_ = std::make_unique<ScanContextDB>();
  if (!sc_db_->load(scan_context_db_file_)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to load .scdb file: %s. Global relocalization disabled. "
      "Node will continue with 3-layer architecture.",
      scan_context_db_file_.c_str());
    sc_db_.reset();
    return;
  }
  // 校验 sc_db 的 config 与节点 config 一致
  if (
    sc_db_->config.num_rings != sc_config_.num_rings ||
    sc_db_->config.num_sectors != sc_config_.num_sectors) {
    RCLCPP_WARN(
      this->get_logger(),
      ".scdb config mismatch (db: %d×%d, node: %d×%d). Global relocalization disabled.",
      sc_db_->config.num_rings, sc_db_->config.num_sectors, sc_config_.num_rings,
      sc_config_.num_sectors);
    sc_db_.reset();
    return;
  }
  global_relocalization_ready_ = true;
  RCLCPP_INFO(
    this->get_logger(), "Global relocalization ready: loaded %zu descriptors from %s",
    sc_db_->descriptors.size(), scan_context_db_file_.c_str());
}

bool SmallGicpRelocalizationNode::checkAndTriggerGlobalRelocalization()
{
  if (!global_relocalization_ready_ || !has_localized_) {
    return false;
  }
  if (!health_monitor_ || !health_monitor_->isUnhealthy()) {
    return false;
  }
  if (global_running_.load()) {
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  double elapsed =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_global_attempt_time_).count() /
    1000.0;
  if (elapsed < global_relocalization_min_interval_sec_) {
    RCLCPP_DEBUG(
      this->get_logger(), "Global relocalization throttled (%.1fs since last attempt)", elapsed);
    return false;
  }

  // 拿当前机器人在 odom 系下的位置：query_local 必须以机器人为原点，
  // 而 result_t_.translation() 是 map 系下 odom 原点位置（语义不同），不能混用。
  Eigen::Vector3d robot_in_odom;
  try {
    auto tf_stamped =
      tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, tf2::TimePointZero);
    robot_in_odom << tf_stamped.transform.translation.x, tf_stamped.transform.translation.y,
      tf_stamped.transform.translation.z;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Global relocalization: failed to lookup %s->%s TF: %s. Skipping this trigger.",
      odom_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    return false;
  }

  // 取累积点云快照
  pcl::PointCloud<pcl::PointXYZ>::Ptr snapshot;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty() || accumulated_count_ < global_accumulated_count_threshold_) {
      RCLCPP_DEBUG(
        this->get_logger(), "Global relocalization: insufficient frames (%d/%d).",
        accumulated_count_, global_accumulated_count_threshold_);
      return false;
    }
    snapshot = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*accumulated_cloud_);
    // 不清 accumulated_cloud，让正常 periodic 流程继续工作
  }

  last_global_attempt_time_ = now;
  global_running_.store(true);
  RCLCPP_WARN(
    this->get_logger(),
    "TRIGGERING global relocalization (health unhealthy, %zu db descriptors, "
    "robot_in_odom=[%.2f, %.2f])",
    sc_db_->descriptors.size(), robot_in_odom.x(), robot_in_odom.y());

  std::thread([this, snapshot, robot_in_odom]() {
    (void)runGlobalRelocalization(snapshot, robot_in_odom);
    global_running_.store(false);
  }).detach();
  return true;
}

bool SmallGicpRelocalizationNode::runGlobalRelocalization(
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_snapshot,
  Eigen::Vector3d robot_in_odom)
{
  if (!accumulated_snapshot || accumulated_snapshot->empty() || !global_relocalization_ready_) {
    return false;
  }

  auto t_start = std::chrono::steady_clock::now();

  // 1. 把 accumulated_snapshot（odom 系）转到机器人局部系作为 query。
  // 局部系原点 = 机器人当前位置（odom 系下的 robot_in_odom），不是 odom 原点。
  // SC 描述子是 yaw 旋转不变量，因此只需消除平移；不需要补偿 yaw（odom 系朝向用作"基准朝向"）。
  pcl::PointCloud<pcl::PointXYZ>::Ptr query_local(new pcl::PointCloud<pcl::PointXYZ>());
  query_local->reserve(accumulated_snapshot->size());
  for (const auto & pt : accumulated_snapshot->points) {
    pcl::PointXYZ p_local;
    p_local.x = pt.x - static_cast<float>(robot_in_odom.x());
    p_local.y = pt.y - static_cast<float>(robot_in_odom.y());
    p_local.z = pt.z;
    query_local->push_back(p_local);
  }

  ScanContext sc_query = sc_engine_->makeScanContext(*query_local);
  RingKey rk_query = sc_engine_->makeRingKey(sc_query);

  // 2. ring key 预筛 top-K
  struct Candidate
  {
    size_t db_idx;
    double rk_dist;
    double sc_dist = 0.0;
    double yaw_shift = 0.0;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(sc_db_->descriptors.size());
  for (size_t i = 0; i < sc_db_->descriptors.size(); ++i) {
    Candidate c;
    c.db_idx = i;
    c.rk_dist = sc_engine_->ringKeyDistance(rk_query, sc_db_->ring_keys[i]);
    candidates.push_back(c);
  }
  size_t top_k = std::min<size_t>(static_cast<size_t>(global_top_k_), candidates.size());
  if (top_k == 0) {
    RCLCPP_WARN(this->get_logger(), "Global relocalization: empty database, skipping.");
    return false;
  }
  std::partial_sort(
    candidates.begin(), candidates.begin() + top_k, candidates.end(),
    [](const Candidate & a, const Candidate & b) { return a.rk_dist < b.rk_dist; });
  candidates.resize(top_k);

  // 3. 对 top-K 算 SC distance，取最小（包含 yaw shift）
  for (auto & c : candidates) {
    auto pair = sc_engine_->distance(sc_query, sc_db_->descriptors[c.db_idx]);
    c.sc_dist = pair.first;
    c.yaw_shift = pair.second;
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return a.sc_dist < b.sc_dist;
  });

  RCLCPP_INFO(
    this->get_logger(),
    "Global relocalization: top-%zu SC matches: best_idx=%zu, sc_dist=%.4f, "
    "candidate_pose=[%.2f, %.2f], yaw_shift=%.3f",
    candidates.size(), candidates[0].db_idx, candidates[0].sc_dist,
    sc_db_->poses[candidates[0].db_idx].x(), sc_db_->poses[candidates[0].db_idx].y(),
    candidates[0].yaw_shift);

  // 4. 对每个候选用 GICP 精调（用主 target_，不是 deep）
  if (!target_ || !target_tree_) {
    RCLCPP_WARN(
      this->get_logger(), "Global relocalization: target map not ready, skipping GICP refinement.");
    return false;
  }
  auto source = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_snapshot, global_registered_leaf_size_);
  if (!source || source->empty()) {
    RCLCPP_WARN(this->get_logger(), "Global relocalization: source empty after downsample.");
    return false;
  }
  for (auto & pt : source->points) {
    pt.z = 0.0f;
  }
  small_gicp::estimate_covariances_omp(*source, num_neighbors_, num_threads_);
  // KdTree on source isn't needed for forward GICP, but we keep variable scope tight.

  auto reg = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  reg->reduction.num_threads = num_threads_;
  reg->rejector.max_dist_sq = global_max_dist_sq_;
  reg->optimizer.max_iterations = global_max_iterations_;

  double best_score = -1.0;
  Eigen::Isometry3d best_T = Eigen::Isometry3d::Identity();
  for (size_t k = 0; k < candidates.size(); ++k) {
    const auto & c = candidates[k];
    const Eigen::Vector3d & db_pose = sc_db_->poses[c.db_idx];
    // GICP 初值 = T_map_odom：把 source（odom 系）平移到 target（map 系）的初始猜测。
    //
    // 推导（已通过 /tmp/global_reloc_smoke 闭环验证）：
    //   query_local 朝向 = odom 朝向；candidate_local 朝向 = map 朝向（codegen yaw=0）。
    //   ScanContextEngine::distance 返回 yaw_shift 满足 query = R_z(yaw_shift) * candidate
    //   （点级关系）。这等价于"query frame 相对 candidate frame 顺时针旋转 yaw_shift"，
    //   而 candidate frame ≡ map frame、query frame ≡ odom frame，所以
    //     R_map_odom = R_z(-yaw_shift)
    //   因此 GICP 初值的 yaw 是 -yaw_shift（不是 +yaw_shift）。
    //
    //   T_map_robot = [R_map_odom | db_pose]
    //   T_odom_robot = [I | robot_in_odom]
    //   T_map_odom = T_map_robot * T_odom_robot^{-1}
    //              = [R_map_odom | db_pose - R_map_odom * robot_in_odom]
    Eigen::Matrix3d R_map_odom =
      Eigen::AngleAxisd(-c.yaw_shift, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d trans = db_pose - R_map_odom * robot_in_odom;
    Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
    guess.translation() << trans.x(), trans.y(), 0.0;
    guess.linear() = R_map_odom;

    auto result = reg->align(*target_, *source, *target_tree_, guess);
    if (!result.converged || result.num_inliers < 50) {
      continue;
    }
    double inlier_ratio = static_cast<double>(result.num_inliers) / source->size();
    double fitness = result.error / static_cast<double>(result.num_inliers);
    double score = static_cast<double>(result.num_inliers) / (fitness + 0.001);
    RCLCPP_INFO(
      this->get_logger(), "Global candidate %zu: inliers=%zu (%.3f), fitness=%.6f, score=%.0f", k,
      result.num_inliers, inlier_ratio, fitness, score);
    if (score > best_score) {
      best_score = score;
      best_T = result.T_target_source;
    }
  }

  auto t_end = std::chrono::steady_clock::now();
  double elapsed_ms = static_cast<double>(
    std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count());

  if (best_score < global_min_score_threshold_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Global relocalization REJECTED in %.0fms: best_score=%.0f < threshold=%.0f", elapsed_ms,
      best_score, global_min_score_threshold_);
    return false;
  }

  // 2D 约束
  Eigen::Vector3d raw_t = best_T.translation();
  Eigen::Matrix3d raw_r = best_T.rotation();
  double final_yaw = std::atan2(raw_r(1, 0), raw_r(0, 0));
  Eigen::Isometry3d constrained = Eigen::Isometry3d::Identity();
  constrained.translation() << raw_t.x(), raw_t.y(), 0.0;
  constrained.linear() = Eigen::AngleAxisd(final_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  double correction = (constrained.translation() - result_t_.translation()).norm();
  if (correction > global_max_correction_distance_) {
    RCLCPP_ERROR(
      this->get_logger(), "Global relocalization REJECTED: correction %.2fm > %.2fm.", correction,
      global_max_correction_distance_);
    return false;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "Global relocalization ACCEPTED in %.0fms: t=[%.3f, %.3f], yaw=%.3f, "
    "correction=%.3fm, score=%.0f",
    elapsed_ms, raw_t.x(), raw_t.y(), final_yaw, correction, best_score);

  result_t_ = previous_result_t_ = constrained;
  if (health_monitor_) {
    health_monitor_->reset();
  }
  notifyTerrainClearing();
  return true;
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
