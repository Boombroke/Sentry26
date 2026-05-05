#include "pointcloud_merger/merger_node.hpp"

#include <memory>

namespace sentry_dual_mid360
{

MergerNode::MergerNode(const rclcpp::NodeOptions & options)
: Node("merger_node", options)
{
  RCLCPP_INFO(this->get_logger(), "sentry_dual_mid360 merger_node started");
}

}  // namespace sentry_dual_mid360

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_dual_mid360::MergerNode>());
  rclcpp::shutdown();
  return 0;
}
