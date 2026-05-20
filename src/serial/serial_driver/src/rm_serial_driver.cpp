// Copyright 2024 Boombroke. Apache-2.0.
//
// v3.0 generic dispatcher implementation.
// See rm_serial_driver.hpp for protocol overview.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

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
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver (protocol v3.0)");
  getParams();

  // Optional: write every TX NavCmd to /tmp/vel_log_*.csv for offline analysis.
  this->declare_parameter<bool>("enable_vel_log", false);
  this->get_parameter("enable_vel_log", enable_vel_log_);
  if (enable_vel_log_) {
    std::string log_path =
      "/tmp/vel_log_" + std::to_string(this->now().nanoseconds()) + ".csv";
    vel_log_file_.open(log_path);
    vel_log_file_ << "timestamp_ns,lx,ly,az,mode\n";
    RCLCPP_INFO(get_logger(), "Velocity logging enabled: %s", log_path.c_str());
  }

  // ----- subscribers -----
  // /cmd_vel_chassis: final chassis Twist after motion_manager arbitration.
  // The driver forwards lx/ly/az into TxNavCmdPacket on every callback.
  nav_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_chassis", 10,
    std::bind(&RMSerialDriver::sendNavCmd, this, std::placeholders::_1));

  // motion_manager/state: optional input. UInt8 mapping: 0=normal, 1=spin_low,
  // 2=spin_high, 3=estop. If no publisher exists yet, current_mode_ stays at
  // 0 (normal), which is the safest fallback.
  motion_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "motion_manager/state", 10,
    [this](const std_msgs::msg::UInt8::SharedPtr msg) { current_mode_ = msg->data; });

  // ----- publishers (split per v3.0 packet semantics) -----
  // RxImuPacket → 2 publishers: gimbal joints (existing topic) +
  //   chassis attitude (new, exposes chassis_pitch/yaw for wheeled-biped TF).
  gimbal_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "serial/gimbal_joint_state", 10);
  chassis_attitude_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "serial/chassis_attitude", 10);

  // RxChassisFeedbackPacket → existing referee/robot_status (HP + ammo) +
  //   new serial/chassis_status (Float32MultiArray = [chassis_power, chassis_mode])
  //   to expose extra fields without modifying rm_interfaces messages.
  robot_status_pub_ = this->create_publisher<rm_interfaces::msg::RobotStatus>(
    "referee/robot_status", 10);
  chassis_status_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "serial/chassis_status", 10);

  // RxRefereePacket → 3 publishers (preserve existing referee/* topic names).
  game_status_pub_ = this->create_publisher<rm_interfaces::msg::GameStatus>(
    "referee/game_status", 10);
  all_hp_pub_ = this->create_publisher<rm_interfaces::msg::GameRobotHP>(
    "referee/all_robot_hp", 10);
  rfid_pub_ = this->create_publisher<rm_interfaces::msg::RfidStatus>(
    "referee/rfidStatus", 10);

  // ----- 1Hz heartbeat timer (TxHeartbeatPacket) -----
  // MCU runs a 1500ms watchdog: if no heartbeat received it forces estop.
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&RMSerialDriver::sendHeartbeat, this));

  // ----- open port + spawn receive thread -----
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
  if (serial_driver_->port() && serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }
  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

// ---------------------------------------------------------------------------
// Generic frame dispatcher.
//
// Algorithm:
//   1. Read 1 byte (HEADER).
//   2. If HEADER is in 0xA0..0xAF (TX echo), drain the rest of the frame
//      using the LEN byte and discard.
//   3. If HEADER is not a known RX header, log throttled WARN and drop the
//      single byte; the next byte may resync.
//   4. Read LEN. Sanity-check against payloadSizeForHeader().
//   5. Read PAYLOAD + CRC16.
//   6. Verify CRC over [HEADER..PAYLOAD]; throttled WARN on failure.
//   7. Dispatch to per-packet handler.
// ---------------------------------------------------------------------------
void RMSerialDriver::receiveLoop()
{
  std::vector<uint8_t> hbuf(1);
  std::vector<uint8_t> lbuf(1);

  while (rclcpp::ok()) {
    try {
      // 1. read HEADER
      serial_driver_->port()->receive(hbuf);
      uint8_t hdr = hbuf[0];

      // 2. drop TX echo: half-duplex / loopback may bounce our own NavCmd back.
      if (isTxHeader(hdr)) {
        serial_driver_->port()->receive(lbuf);
        uint8_t plen = lbuf[0];
        std::vector<uint8_t> drain(plen + 2);
        serial_driver_->port()->receive(drain);
        RCLCPP_DEBUG(
          get_logger(), "Skipped TX echo 0x%02X (%uB payload)", hdr,
          static_cast<unsigned>(plen));
        continue;
      }

      // 3. unknown header — single byte drop, retry on next loop.
      if (!isRxHeader(hdr)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Unknown header 0x%02X, dropping byte", hdr);
        continue;
      }

      // 4. read LEN, verify
      serial_driver_->port()->receive(lbuf);
      uint8_t plen = lbuf[0];
      const size_t expected_payload = payloadSizeForHeader(hdr);
      if (plen != expected_payload) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LEN mismatch on 0x%02X: got %u want %zu", hdr, static_cast<unsigned>(plen),
          expected_payload);
        continue;
      }

      // 5. read PAYLOAD + CRC16
      std::vector<uint8_t> body(plen + 2);
      serial_driver_->port()->receive(body);

      // 6. CRC over the full frame [HEADER][LEN][PAYLOAD][CRC16]
      std::vector<uint8_t> full;
      full.reserve(plen + 4);
      full.push_back(hdr);
      full.push_back(plen);
      full.insert(full.end(), body.begin(), body.end());

      if (!crc16::Verify_CRC16_Check_Sum(full.data(), full.size())) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "CRC failed on header 0x%02X (%zuB)", hdr, full.size());
        continue;
      }

      // 7. dispatch
      switch (hdr) {
        case HDR_RX_IMU:
          handleImu(fromVector<RxImuPacket>(full));
          break;
        case HDR_RX_CHASSIS_FEEDBACK:
          handleChassisFeedback(fromVector<RxChassisFeedbackPacket>(full));
          break;
        case HDR_RX_REFEREE:
          handleReferee(fromVector<RxRefereePacket>(full));
          break;
        default:
          // already filtered by isRxHeader; unreachable.
          break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Receive error: %s", ex.what());
      reopenPort();
    }
  }
}

// ---------------------------------------------------------------------------
// RX handlers — convert packed wire structs to ROS messages.
// ---------------------------------------------------------------------------
void RMSerialDriver::handleImu(const RxImuPacket & pkt)
{
  const auto stamp = now();

  sensor_msgs::msg::JointState gimbal;
  gimbal.header.stamp = stamp;
  gimbal.name = {"gimbal_pitch_joint", "gimbal_yaw_joint"};
  gimbal.position = {pkt.gimbal_pitch, pkt.gimbal_yaw};
  gimbal_joint_pub_->publish(gimbal);

  sensor_msgs::msg::JointState chassis;
  chassis.header.stamp = stamp;
  chassis.name = {"chassis_pitch", "chassis_yaw"};
  chassis.position = {pkt.chassis_pitch, pkt.chassis_yaw};
  chassis_attitude_pub_->publish(chassis);
}

void RMSerialDriver::handleChassisFeedback(const RxChassisFeedbackPacket & pkt)
{
  rm_interfaces::msg::RobotStatus status;
  status.current_hp = pkt.current_hp;
  status.projectile_allowance_17mm = pkt.projectile_allowance_17mm;
  robot_status_pub_->publish(status);

  // Expose extra fields (chassis_power, chassis_mode) without expanding
  // the upstream rm_interfaces/RobotStatus message.
  std_msgs::msg::Float32MultiArray chassis_extra;
  chassis_extra.data = {pkt.chassis_power, static_cast<float>(pkt.chassis_mode)};
  chassis_status_pub_->publish(chassis_extra);
}

void RMSerialDriver::handleReferee(const RxRefereePacket & pkt)
{
  rm_interfaces::msg::GameStatus game;
  game.game_progress = pkt.game_progress;
  game.stage_remain_time = pkt.stage_remain_time;
  game_status_pub_->publish(game);

  rm_interfaces::msg::GameRobotHP hp;
  hp.ally_1_robot_hp = pkt.ally_1_robot_hp;
  hp.ally_2_robot_hp = pkt.ally_2_robot_hp;
  hp.ally_3_robot_hp = pkt.ally_3_robot_hp;
  hp.ally_4_robot_hp = pkt.ally_4_robot_hp;
  hp.ally_7_robot_hp = pkt.ally_7_robot_hp;
  hp.ally_outpost_hp = pkt.ally_outpost_hp;
  hp.ally_base_hp = pkt.ally_base_hp;
  hp.event_data = pkt.event_data;
  all_hp_pub_->publish(hp);

  rm_interfaces::msg::RfidStatus rfid;
  rfid.friendly_supply_zone_non_exchange = pkt.rfid_base;
  rfid_pub_->publish(rfid);
}

// ---------------------------------------------------------------------------
// TX — NavCmd (event-driven, on each /cmd_vel_chassis callback).
// ---------------------------------------------------------------------------
void RMSerialDriver::sendNavCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    TxNavCmdPacket pkt;  // header/len defaulted by struct definition
    pkt.lx       = static_cast<float>(msg->linear.x);
    pkt.ly       = static_cast<float>(msg->linear.y);
    pkt.az       = static_cast<float>(msg->angular.z);
    pkt.mode     = current_mode_;
    pkt.reserved = 0;

    // CRC over [header..payload] = first (sizeof(pkt) - 2) bytes; the helper
    // writes the trailing CRC16 in-place.
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));

    if (enable_vel_log_ && vel_log_file_.is_open()) {
      vel_log_file_ << this->now().nanoseconds() << ',' << pkt.lx << ',' << pkt.ly << ','
                    << pkt.az << ',' << static_cast<int>(pkt.mode) << '\n';
    }

    auto data = toVector(pkt);
    serial_driver_->port()->send(data);

    RCLCPP_DEBUG(
      get_logger(), "TX NavCmd lx=%.3f ly=%.3f az=%.3f mode=%u (%zuB)",
      pkt.lx, pkt.ly, pkt.az, static_cast<unsigned>(pkt.mode), data.size());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Error while sending NavCmd: %s", ex.what());
    reopenPort();
  }
}

// ---------------------------------------------------------------------------
// TX — Heartbeat (1Hz wall-clock timer).
// ---------------------------------------------------------------------------
void RMSerialDriver::sendHeartbeat()
{
  try {
    if (!serial_driver_->port() || !serial_driver_->port()->is_open()) {
      return;
    }
    TxHeartbeatPacket pkt;
    pkt.ros_state = ros_state_;
    pkt.reserved  = 0;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
    auto data = toVector(pkt);
    serial_driver_->port()->send(data);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Heartbeat send failed: %s", ex.what());
  }
}

// ---------------------------------------------------------------------------
// Boilerplate — params + reopen
// ---------------------------------------------------------------------------
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
