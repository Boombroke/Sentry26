#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"

#include "rm_serial_driver/packet.hpp"

#include <fstream>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);
  ~RMSerialDriver() override;

private:
  void getParams();
  void receiveLoop();
  void sendNavData(const geometry_msgs::msg::Twist::SharedPtr msg);
  void reopenPort();

  void handleImuPacket(const ReceiveImuPacket & pkt);
  void handleStatusPacket(const ReceiveStatusPacket & pkt);
  void handleHpPacket(const ReceiveHpPacket & pkt);

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameRobotHP>::SharedPtr allHP_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RfidStatus>::SharedPtr rfid_pub_;

  std::thread receive_thread_;

  bool enable_vel_log_{false};
  std::ofstream vel_log_file_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
