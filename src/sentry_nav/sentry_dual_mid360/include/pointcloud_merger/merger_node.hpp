#ifndef SENTRY_DUAL_MID360__POINTCLOUD_MERGER__MERGER_NODE_HPP_
#define SENTRY_DUAL_MID360__POINTCLOUD_MERGER__MERGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace sentry_dual_mid360
{

class MergerNode : public rclcpp::Node
{
public:
  explicit MergerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace sentry_dual_mid360

#endif  // SENTRY_DUAL_MID360__POINTCLOUD_MERGER__MERGER_NODE_HPP_
