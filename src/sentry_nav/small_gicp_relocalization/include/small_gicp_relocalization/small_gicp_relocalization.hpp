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

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp_relocalization/localization_health.hpp"
#include "small_gicp_relocalization/scan_context.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void periodicRegistrationCallback();
  void loadPcdFile(const std::string & file_name);
  void prepareTargetMap();
  void prepareDeepTargetMap();
  bool performRegistration(bool is_periodic);
  bool performEmergencyRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void notifyTerrainClearing();
  void deepVerificationTimerCallback();
  void runDeepVerification(
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_snapshot, Eigen::Isometry3d initial_guess);
  void prepareScanContextDB();
  bool checkAndTriggerGlobalRelocalization();
  bool runGlobalRelocalization(
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_snapshot,
    Eigen::Vector3d robot_in_odom);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  int num_threads_;
  int num_neighbors_;
  int max_iterations_;
  int accumulated_count_threshold_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  double min_range_;

  double min_inlier_ratio_;
  double max_fitness_error_;

  bool enable_periodic_relocalization_;
  double relocalization_interval_;
  double max_correction_distance_;
  double emergency_max_dist_sq_;
  int emergency_consecutive_failures_;
  double emergency_max_correction_distance_{3.0};
  double emergency_min_score_threshold_{200000.0};
  double quality_convergence_threshold_{0.008};

  int accumulated_count_;
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;
  rclcpp::TimerBase::SharedPtr deep_timer_;

  // Deep verification (low-frequency, high-quality) layer
  bool enable_deep_verification_{false};
  double deep_verification_interval_{30.0};
  int deep_accumulated_count_threshold_{100};
  double deep_max_dist_sq_{225.0};
  int deep_max_iterations_{300};
  float deep_global_leaf_size_{0.05f};
  float deep_registered_leaf_size_{0.05f};
  double deep_min_inlier_ratio_{0.3};
  double deep_max_fitness_error_{0.5};
  double deep_max_correction_distance_{15.0};
  double deep_min_score_threshold_{500000.0};

  pcl::PointCloud<pcl::PointCovariance>::Ptr target_deep_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_deep_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    deep_register_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr deep_accumulated_cloud_;
  int deep_accumulated_count_{0};
  std::mutex deep_cloud_mutex_;
  std::atomic<bool> deep_running_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr map_clearing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cloud_clearing_pub_;

  bool global_map_ready_ = false;
  bool has_localized_ = false;
  int consecutive_periodic_failures_ = 0;
  Eigen::Isometry3d accumulation_snapshot_t_ = Eigen::Isometry3d::Identity();
  double terrain_clearing_threshold_;

  std::mutex cloud_mutex_;

  // Localization health monitor (T3): sliding-window score median + stale check
  std::unique_ptr<LocalizationHealth> health_monitor_;
  size_t health_window_size_{5};
  double health_unhealthy_score_threshold_{50000.0};
  double health_stale_timeout_sec_{30.0};

  // Global relocalization (Scan Context) layer
  bool enable_global_relocalization_{false};
  std::string scan_context_db_file_;
  ScanContextConfig sc_config_;
  int global_top_k_{5};
  double global_relocalization_min_interval_sec_{30.0};
  double global_max_correction_distance_{30.0};
  double global_min_score_threshold_{300000.0};
  int global_accumulated_count_threshold_{50};
  double global_max_dist_sq_{225.0};
  int global_max_iterations_{200};
  float global_registered_leaf_size_{0.05f};

  std::unique_ptr<ScanContextEngine> sc_engine_;
  std::unique_ptr<ScanContextDB> sc_db_;
  bool global_relocalization_ready_{false};
  std::atomic<bool> global_running_{false};
  std::chrono::steady_clock::time_point last_global_attempt_time_;

  // Manual override: once /initialpose received, all automatic relocalization
  // paths (periodic / emergency / deep / cold-start SC / health-triggered SC)
  // are disabled until node restart. Supports multiple manual updates.
  std::atomic<bool> manual_pose_locked_{false};
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
