#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
  getParams();

  this->declare_parameter<bool>("enable_vel_log", false);
  this->get_parameter("enable_vel_log", enable_vel_log_);
  if (enable_vel_log_) {
    std::string log_path = "/tmp/vel_log_" +
      std::to_string(this->now().nanoseconds()) + ".csv";
    vel_log_file_.open(log_path);
    vel_log_file_ << "timestamp_ns,vel_x,vel_y,vel_w\n";
    RCLCPP_INFO(get_logger(), "Velocity logging enabled: %s", log_path.c_str());
  }

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&RMSerialDriver::sendNavData, this, std::placeholders::_1));

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "serial/gimbal_joint_state", 10);
  game_status_pub_ = this->create_publisher<rm_interfaces::msg::GameStatus>(
    "referee/game_status", 10);
  robot_status_pub_ = this->create_publisher<rm_interfaces::msg::RobotStatus>(
    "referee/robot_status", 10);
  allHP_pub_ = this->create_publisher<rm_interfaces::msg::GameRobotHP>(
    "referee/all_robot_hp", 10);
  rfid_pub_ = this->create_publisher<rm_interfaces::msg::RfidStatus>(
    "referee/rfidStatus", 10);

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveLoop, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
}

RMSerialDriver::~RMSerialDriver()
{
  if (vel_log_file_.is_open()) {
    vel_log_file_.close();
  }
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }
  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveLoop()
{
  std::vector<uint8_t> header_buf(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceiveHpPacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header_buf);
      uint8_t hdr = header_buf[0];

      size_t pkt_size = packetSizeForHeader(hdr);
      if (pkt_size == 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Invalid header: 0x%02X", hdr);
        continue;
      }

      data.resize(pkt_size - 1);
      serial_driver_->port()->receive(data);
      data.insert(data.begin(), hdr);

      bool crc_ok = crc16::Verify_CRC16_Check_Sum(data.data(), data.size());
      if (!crc_ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "CRC error on packet 0x%02X", hdr);
        continue;
      }

      RCLCPP_DEBUG(get_logger(), "RX packet 0x%02X (%zu bytes) CRC OK", hdr, data.size());

      switch (hdr) {
        case HEADER_IMU:
          handleImuPacket(fromVector<ReceiveImuPacket>(data));
          break;
        case HEADER_STATUS:
          handleStatusPacket(fromVector<ReceiveStatusPacket>(data));
          break;
        case HEADER_HP:
          handleHpPacket(fromVector<ReceiveHpPacket>(data));
          break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::handleImuPacket(const ReceiveImuPacket & pkt)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  msg.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
  msg.position = {pkt.pitch, pkt.yaw};
  joint_state_pub_->publish(msg);
}

void RMSerialDriver::handleStatusPacket(const ReceiveStatusPacket & pkt)
{
  rm_interfaces::msg::GameStatus game;
  game.game_progress = pkt.game_progress;
  game.stage_remain_time = pkt.stage_remain_time;
  game_status_pub_->publish(game);

  rm_interfaces::msg::RobotStatus robot;
  robot.current_hp = pkt.current_hp;
  robot.projectile_allowance_17mm = pkt.projectile_allowance_17mm;
  robot_status_pub_->publish(robot);

  rm_interfaces::msg::RfidStatus rfid;
  rfid.friendly_supply_zone_non_exchange = pkt.rfid_base;
  rfid_pub_->publish(rfid);
}

void RMSerialDriver::handleHpPacket(const ReceiveHpPacket & pkt)
{
  rm_interfaces::msg::GameRobotHP hp;
  hp.ally_1_robot_hp = pkt.ally_1_robot_hp;
  hp.ally_2_robot_hp = pkt.ally_2_robot_hp;
  hp.ally_3_robot_hp = pkt.ally_3_robot_hp;
  hp.ally_4_robot_hp = pkt.ally_4_robot_hp;
  hp.ally_7_robot_hp = pkt.ally_7_robot_hp;
  hp.ally_outpost_hp = pkt.ally_outpost_hp;
  hp.ally_base_hp    = pkt.ally_base_hp;
  allHP_pub_->publish(hp);
}

void RMSerialDriver::sendNavData(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    SendNavPacket packet;
    packet.vel_x = msg->linear.x;
    packet.vel_y = msg->linear.y;
    packet.vel_w = msg->angular.z;

    if (enable_vel_log_ && vel_log_file_.is_open()) {
      vel_log_file_ << this->now().nanoseconds() << ','
                    << packet.vel_x << ',' << packet.vel_y << ',' << packet.vel_w << '\n';
    }

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);

    RCLCPP_DEBUG(
      get_logger(), "TX Nav: vx=%.3f vy=%.3f vw=%.3f (%zu bytes)",
      packet.vel_x, packet.vel_y, packet.vel_w, data.size());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");
    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");
    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");
    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  while (rclcpp::ok()) {
    try {
      if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
      return;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
