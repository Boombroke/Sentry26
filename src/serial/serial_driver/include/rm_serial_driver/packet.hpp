#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm_serial_driver
{

constexpr uint8_t FH_RX = 0x5B;
constexpr uint8_t FH_TX = 0xB5;

// 电控→ROS，38 bytes
struct SendPacket
{
  uint8_t  header = FH_RX;
  float    pitch;
  float    yaw;
  uint8_t  game_progress; //0=未开始, 1=准备阶段, 2=自检阶段, 3=5s比赛倒计时, 4=比赛中, 5=比赛结束
  uint16_t stage_remain_time;
  uint16_t current_hp;
  uint16_t project_allowance_17mm;
  uint8_t  team_colour;
  uint8_t  rfid_base;
  uint16_t ally_1_robot_HP;
  uint16_t ally_2_robot_HP;
  uint16_t ally_3_robot_HP;
  uint16_t ally_4_robot_HP;
  uint16_t ally_7_robot_HP;
  uint16_t ally_outpost_HP;
  uint16_t ally_base_hp;
  uint32_t event_data;  // 1=己方增益点, 2=己方堡垒被敌方占领
  uint16_t checksum = 0;
} __attribute__((packed));

// ROS→电控，15 bytes
struct ReceivePacket
{
  uint8_t  header = FH_TX;
  float    lx;
  float    ly;
  float    az;
  uint16_t checksum = 0;
} __attribute__((packed));

static_assert(sizeof(SendPacket) == 38, "SendPacket wire size must be 38 bytes");
static_assert(sizeof(ReceivePacket) == 15, "ReceivePacket wire size must be 15 bytes");

inline size_t packetSizeForHeader(uint8_t header)
{
  switch (header) {
    case FH_RX:
      return sizeof(SendPacket);
    case FH_TX:
      return sizeof(ReceivePacket);
    default:
      return 0;
  }
}

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::memcpy(reinterpret_cast<uint8_t *>(&packet), data.data(),
              std::min(data.size(), sizeof(T)));
  return packet;
}

inline std::vector<uint8_t> toVector(const ReceivePacket & data)
{
  std::vector<uint8_t> packet(sizeof(ReceivePacket));
  std::memcpy(packet.data(), reinterpret_cast<const uint8_t *>(&data), sizeof(ReceivePacket));
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
