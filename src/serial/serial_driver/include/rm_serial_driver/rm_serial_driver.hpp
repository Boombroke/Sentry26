// Copyright 2024 Boombroke. Apache-2.0.
//
// rm_serial_driver — ROS 2 driver for the v3.0 multi-packet serial protocol.
//   - Frame: [HEADER 1B][LEN 1B][PAYLOAD N B][CRC16 2B], total = N + 4
//   - RX (MCU→ROS): 0x51 Imu/200Hz, 0x52 ChassisFeedback/20Hz, 0x53 Referee/1Hz
//   - TX (ROS→MCU): 0xA2 NavCmd/20Hz (event-driven), 0xA3 Heartbeat/1Hz timer

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"

#include "rm_serial_driver/packet.hpp"

#include <cstdint>
#include <fstream>
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
  // ---------- lifecycle ----------
  void getParams();
  void receiveLoop();
  void reopenPort();

  // ---------- TX (ROS → MCU) ----------
  void sendNavCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendHeartbeat();

  // ---------- RX dispatchers ----------
  void handleImu(const RxImuPacket & pkt);
  void handleChassisFeedback(const RxChassisFeedbackPacket & pkt);
  void handleReferee(const RxRefereePacket & pkt);

  // ---------- IO state ----------
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::thread receive_thread_;

  // ---------- subscribers ----------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr motion_state_sub_;

  // ---------- publishers (split per v3.0 packet) ----------
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr chassis_attitude_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr chassis_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameRobotHP>::SharedPtr all_hp_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RfidStatus>::SharedPtr rfid_pub_;

  // ---------- 1Hz heartbeat timer ----------
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // ---------- runtime state ----------
  // current_mode_ is forwarded into TxNavCmdPacket.mode every send.
  // Updated by motion_manager/state subscription. Defaults to 0 (normal).
  uint8_t current_mode_{0};
  // ros_state_ is sent in heartbeats. Defaults to RUNNING (=2).
  uint8_t ros_state_{2};

  // ---------- vel debug log (preserved feature) ----------
  bool enable_vel_log_{false};
  std::ofstream vel_log_file_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
