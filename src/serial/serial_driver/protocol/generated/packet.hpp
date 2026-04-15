#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm_serial_driver
{

constexpr uint8_t HEADER_IMU       = 0xA1;
constexpr uint8_t HEADER_STATUS    = 0xA2;
constexpr uint8_t HEADER_HP        = 0xA3;
constexpr uint8_t HEADER_NAV_TX    = 0xB5;


// 0xA1 — 11 bytes, ~1000Hz
struct ReceiveImuPacket
{
  uint8_t  header = HEADER_IMU;
float    pitch;                     //云台 pitch
float    yaw;                       //云台 yaw
uint16_t checksum = 0;
} __attribute__((packed));

// 0xA2 — 12 bytes, ~10Hz
struct ReceiveStatusPacket
{
  uint8_t  header = HEADER_STATUS;
uint8_t  game_progress;             //游戏阶段 0-未开始 1-准备 2-自检 3-倒计时 4-比赛中 5-结算
uint16_t stage_remain_time;         //当前阶段剩余时间
uint16_t current_hp;                //机器人当前血量
uint16_t projectile_allowance_17mm; //17mm弹丸剩余发射次数
uint8_t  team_colour;               //1=red 0=blue
uint8_t  rfid_base;                 //己方基地增益点
uint16_t checksum = 0;
} __attribute__((packed));

// 0xA3 — 17 bytes, ~2Hz
struct ReceiveHpPacket
{
  uint8_t  header = HEADER_HP;
uint16_t ally_1_robot_hp;           //己方1号英雄血量
uint16_t ally_2_robot_hp;           //己方2号工程血量
uint16_t ally_3_robot_hp;           //己方3号步兵血量
uint16_t ally_4_robot_hp;           //己方4号步兵血量
uint16_t ally_7_robot_hp;           //己方7号哨兵血量
uint16_t ally_outpost_hp;           //己方前哨站血量
uint16_t ally_base_hp;              //己方基地血量
uint16_t checksum = 0;
} __attribute__((packed));


// 0xB5 — 15 bytes, ROS→电控导航速度指令
struct SendNavPacket
{
  uint8_t  header = HEADER_NAV_TX;
float    vel_x;                     //底盘x方向线速度
float    vel_y;                     //底盘y方向线速度
float    vel_w;                     //底盘角速度
uint16_t checksum = 0;
} __attribute__((packed));

inline size_t packetSizeForHeader(uint8_t header)
{
  switch (header) {
case HEADER_IMU:    return sizeof(ReceiveImuPacket);
case HEADER_STATUS:    return sizeof(ReceiveStatusPacket);
case HEADER_HP:    return sizeof(ReceiveHpPacket);

    default:                 return 0;
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

inline std::vector<uint8_t> toVector(const SendNavPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendNavPacket));
  std::memcpy(packet.data(), reinterpret_cast<const uint8_t *>(&data), sizeof(SendNavPacket));
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
