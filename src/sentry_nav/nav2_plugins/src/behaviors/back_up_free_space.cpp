// Copyright 2024 Polaris Xia
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

#include "nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_behaviors_plugins
{

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(
    node, "min_backward_projection", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, "max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, "turn_gain", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "min_escape_clearance", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, "unknown_cost_threshold", rclcpp::ParameterValue(253.0));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("min_backward_projection", min_backward_projection_);
  node->get_parameter("max_angular_vel", max_angular_vel_);
  node->get_parameter("turn_gain", turn_gain_);
  node->get_parameter("min_escape_clearance", min_escape_clearance_);
  node->get_parameter("unknown_cost_threshold", unknown_cost_threshold_);
  node->get_parameter("visualize", visualize_);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

nav2_behaviors::ResultStatus BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }

  // get costmap
  auto costmap = result.get()->map;
  costmap_frame_ = costmap.header.frame_id.empty() ? global_frame_ : costmap.header.frame_id;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }

  geometry_msgs::msg::PoseStamped local_initial_pose;
  if (!nav2_util::getCurrentPose(
        local_initial_pose, *tf_, local_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial local robot pose is not available.");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }

  geometry_msgs::msg::Pose2D global_pose;
  global_pose.x = initial_pose_.pose.position.x;
  global_pose.y = initial_pose_.pose.position.y;
  global_pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

  // Search in the global costmap frame, but run collision checks in local_frame_.
  // The local checker is backed by local_costmap/costmap_raw and cannot safely consume map-frame poses
  // when map->odom has been corrected by relocalization.
  const float best_angle =
    findBestBackwardDirection(costmap, global_pose, max_radius_, M_PI / 48.0);
  const float forward_relative_angle =
    angles::shortest_angular_distance(global_pose.theta, best_angle);
  const float backward_error =
    angles::shortest_angular_distance(global_pose.theta + M_PI, best_angle);
  const float backward_projection = std::max(0.0f, std::cos(backward_error));
  const float escape_clearance = scoreDirection(costmap, global_pose, best_angle, max_radius_);

  if (backward_projection < min_backward_projection_ || escape_clearance < min_escape_clearance_) {
    RCLCPP_WARN(
      logger_,
      "No sufficiently free backward sector found (best=%.3f, forward_relative=%.3f, "
      "backward_error=%.3f, projection=%.3f, clearance=%.3f), refusing blind backup",
      best_angle, forward_relative_angle, backward_error, backward_projection, escape_clearance);
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }

  twist_x_ = -std::fabs(command->speed);
  twist_y_ = 0.0;
  twist_theta_ = std::clamp(
    static_cast<double>(turn_gain_ * backward_error), -max_angular_vel_, max_angular_vel_);
  command_x_ = -std::fabs(command->target.x);
  command_time_allowance_ = command->time_allowance;
  distance_traveled_ = 0.0;

  end_time_ = clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }
  last_pose_ = initial_pose_;
  RCLCPP_WARN(
    logger_,
    "backing up %.3f meters with speed %.3f and wz %.3f, free-space angle %.3f, forward relative "
    "%.3f, backward error %.3f, projection %.3f, clearance %.3f",
    std::fabs(command_x_), twist_x_, twist_theta_, best_angle, forward_relative_angle,
    backward_error, backward_projection, escape_clearance);

  return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED};
}

nav2_behaviors::ResultStatus BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, costmap_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }

  geometry_msgs::msg::PoseStamped local_current_pose;
  if (!nav2_util::getCurrentPose(
        local_current_pose, *tf_, local_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current local robot pose is not available.");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  (void)diff_x;
  (void)diff_y;

  const double step_dx = current_pose.pose.position.x - last_pose_.pose.position.x;
  const double step_dy = current_pose.pose.position.y - last_pose_.pose.position.y;
  distance_traveled_ += std::hypot(step_dx, step_dy);
  last_pose_ = current_pose;

  feedback_->distance_traveled = distance_traveled_;
  action_server_->publish_feedback(feedback_);

  if (distance_traveled_ >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::SUCCEEDED};
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = clock_->now();
  cmd_vel->header.frame_id = robot_base_frame_;
  cmd_vel->twist.linear.y = twist_y_;
  cmd_vel->twist.linear.x = twist_x_;
  cmd_vel->twist.angular.z = twist_theta_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = local_current_pose.pose.position.x;
  pose.y = local_current_pose.pose.position.y;
  pose.theta = tf2::getYaw(local_current_pose.pose.orientation);

  if (!isArcCollisionFree(std::fabs(command_x_) - distance_traveled_, cmd_vel->twist, pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting BackUpFreeSpace");
    return nav2_behaviors::ResultStatus{nav2_behaviors::Status::FAILED};
  }

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::ResultStatus{nav2_behaviors::Status::RUNNING};
}

bool BackUpFreeSpace::isArcCollisionFree(
  double remaining_distance, const geometry_msgs::msg::Twist & cmd_vel,
  geometry_msgs::msg::Pose2D pose)
{
  if (remaining_distance <= 0.0) {
    return true;
  }

  const int max_cycle_count = static_cast<int>(cycle_frequency_ * simulate_ahead_time_);
  if (max_cycle_count <= 0) {
    return true;
  }

  const double dt = 1.0 / cycle_frequency_;
  double simulated_distance = 0.0;
  bool fetch_data = true;

  for (int i = 0; i < max_cycle_count; ++i) {
    pose.theta = angles::normalize_angle(pose.theta + cmd_vel.angular.z * dt);
    pose.x += cmd_vel.linear.x * std::cos(pose.theta) * dt;
    pose.y += cmd_vel.linear.x * std::sin(pose.theta) * dt;
    simulated_distance += std::fabs(cmd_vel.linear.x) * dt;

    if (!local_collision_checker_->isCollisionFree(pose, fetch_data)) {
      return false;
    }
    fetch_data = false;

    if (simulated_distance >= remaining_distance) {
      break;
    }
  }

  return true;
}

float BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment)
{
  float best_angle = start_angle;

  float first_safe_angle = -1.0f;
  float last_unsafe_angle = -1.0f;

  float final_safe_angle = 0.0f;
  float final_unsafe_angle = 0.0f;

  float resolution = costmap.metadata.resolution;
  float origin_x = costmap.metadata.origin.position.x;
  float origin_y = costmap.metadata.origin.position.y;
  int size_x = costmap.metadata.size_x;
  int size_y = costmap.metadata.size_y;

  float map_min_x = origin_x;
  float map_max_x = origin_x + (size_x * resolution);
  float map_min_y = origin_y;
  float map_max_y = origin_y + (size_y * resolution);

  for (float angle = start_angle; angle <= end_angle; angle += angle_increment) {
    bool is_safe = true;

    for (float r = 0; r <= radius; r += resolution) {
      float x = pose.x + r * std::cos(angle);
      float y = pose.y + r * std::sin(angle);

      if (x >= map_min_x && x <= map_max_x && y >= map_min_y && y <= map_max_y) {
        int i = static_cast<int>((x - origin_x) / resolution);
        int j = static_cast<int>((y - origin_y) / resolution);

        if (i >= 0 && i < size_x && j >= 0 && j < size_y) {
          if (costmap.data[i + j * size_x] >= 253) {
            is_safe = false;
            break;
          }
        } else {
          is_safe = false;
          break;
        }
      } else {
        is_safe = false;
        break;
      }
    }
    if (is_safe && first_safe_angle == -1.0f) {
      first_safe_angle = angle;
    }

    if (!is_safe && first_safe_angle != -1.0f && last_unsafe_angle == -1.0f) {
      last_unsafe_angle = angle;
    }

    if (
      last_unsafe_angle - first_safe_angle > final_unsafe_angle - final_safe_angle &&
      first_safe_angle != -1.0f && last_unsafe_angle != -1.0f) {
      final_safe_angle = first_safe_angle;
      final_unsafe_angle = last_unsafe_angle;
      first_safe_angle = -1.0f;
      last_unsafe_angle = -1.0f;
    }
  }

  if (first_safe_angle != -1.0f && last_unsafe_angle == -1.0f) {
    last_unsafe_angle = end_angle;
  }

  if (
    last_unsafe_angle - first_safe_angle > final_unsafe_angle - final_safe_angle &&
    first_safe_angle != -1.0f && last_unsafe_angle != -1.0f) {
    final_safe_angle = first_safe_angle;
    final_unsafe_angle = last_unsafe_angle;
  }

  best_angle = (final_safe_angle + final_unsafe_angle) / 2.0f;

  if (visualize_) {
    visualize(pose, radius, final_safe_angle, final_unsafe_angle);
  }

  return best_angle;
}

float BackUpFreeSpace::findBestBackwardDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius,
  float angle_increment)
{
  float best_angle = angles::normalize_angle(pose.theta + M_PI);
  float best_score = -1.0f;

  for (float relative = -M_PI; relative <= M_PI; relative += angle_increment) {
    const float backward_projection = std::max(0.0f, -std::cos(relative));
    if (backward_projection < min_backward_projection_) {
      continue;
    }

    const float angle = angles::normalize_angle(pose.theta + relative);
    const float clearance = scoreDirection(costmap, pose, angle, radius);
    if (clearance < min_escape_clearance_) {
      continue;
    }

    // Prefer long obstacle-free rays, but bias toward directions the differential base can execute
    // as a backward arc without demanding excessive in-place spin.
    const float score = clearance * (0.5f + 0.5f * backward_projection);
    if (score > best_score) {
      best_score = score;
      best_angle = angle;
    }
  }

  if (visualize_) {
    visualize(pose, radius, best_angle - angle_increment, best_angle + angle_increment);
  }

  return best_angle;
}

float BackUpFreeSpace::scoreDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float angle,
  float radius)
{
  const float resolution = costmap.metadata.resolution;
  const float origin_x = costmap.metadata.origin.position.x;
  const float origin_y = costmap.metadata.origin.position.y;
  const int size_x = costmap.metadata.size_x;
  const int size_y = costmap.metadata.size_y;

  if (resolution <= 0.0f || size_x <= 0 || size_y <= 0) {
    return 0.0f;
  }

  const float map_min_x = origin_x;
  const float map_max_x = origin_x + (size_x * resolution);
  const float map_min_y = origin_y;
  const float map_max_y = origin_y + (size_y * resolution);

  float clearance = 0.0f;
  for (float r = 0.0f; r <= radius; r += resolution) {
    const float x = pose.x + r * std::cos(angle);
    const float y = pose.y + r * std::sin(angle);

    if (x < map_min_x || x >= map_max_x || y < map_min_y || y >= map_max_y) {
      break;
    }

    const int i = static_cast<int>((x - origin_x) / resolution);
    const int j = static_cast<int>((y - origin_y) / resolution);
    if (i < 0 || i >= size_x || j < 0 || j >= size_y) {
      break;
    }

    const int lethal_threshold = std::clamp(static_cast<int>(unknown_cost_threshold_), 0, 255);
    const int cost = costmap.data[i + j * size_x];
    if (cost >= lethal_threshold) {
      break;
    }
    clearance = r;
  }

  return clearance;
}

std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_unsafe_angle, 2, 0.0f, 0.0f, 1.0f));

  const float best_angle = (first_safe_angle + last_unsafe_angle) / 2.0f;
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace nav2_behaviors_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors_plugins::BackUpFreeSpace, nav2_core::Behavior)
