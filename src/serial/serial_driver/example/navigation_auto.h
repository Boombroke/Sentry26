#ifndef __NAVIGATION_AUTO_H
#define __NAVIGATION_AUTO_H

#include "cmsis_os.h"
#include <stdbool.h>
#include <string.h>

#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

#define FH_TX_IMU      0xA1
#define FH_TX_STATUS   0xA2
#define FH_TX_HP       0xA3

#define FH_RX          0xB5

// FreeRTOS task ~1000Hz (osDelay(1)). Interval in ticks.
#define SEND_IMU_INTERVAL      1
#define SEND_STATUS_INTERVAL   100
#define SEND_HP_INTERVAL       500


// 0xA1 — 11 bytes, ~1000Hz
typedef struct
{
  uint8_t  header;
float    pitch;
float    yaw;
uint16_t checksum;
} __attribute__((packed)) SendImuPacket;

// 0xA2 — 12 bytes, ~10Hz
typedef struct
{
  uint8_t  header;
uint8_t  game_progress;
uint16_t stage_remain_time;
uint16_t current_hp;
uint16_t projectile_allowance_17mm;
uint8_t  team_colour;  // 1=red 0=blue
uint8_t  rfid_base;
uint16_t checksum;
} __attribute__((packed)) SendStatusPacket;

// 0xA3 — 17 bytes, ~2Hz
typedef struct
{
  uint8_t  header;
uint16_t ally_1_robot_hp;
uint16_t ally_2_robot_hp;
uint16_t ally_3_robot_hp;
uint16_t ally_4_robot_hp;
uint16_t ally_7_robot_hp;
uint16_t ally_outpost_hp;
uint16_t ally_base_hp;
uint16_t checksum;
} __attribute__((packed)) SendHpPacket;


typedef struct
{
  uint8_t  header;
float    vel_x;
float    vel_y;
float    vel_w;
uint16_t checksum;
} __attribute__((packed)) ReceivePacket;

typedef struct
{
float    vel_x;
float    vel_y;
float    vel_w;
} Navigation;

void Navigation_Init(void);
void Navigation_Task(void);

void send_ImuPacket(void);
void send_StatusPacket(void);
void send_HpPacket(void);

void receive_Navigation(void);
void Navigation_Parm(Navigation *navigation);
void Append_CRC16_Check_Sum_Buf(uint8_t *buf, uint32_t len);

extern SendImuPacket    Smsg_imu  ;
extern SendStatusPacket Smsg_status;
extern SendHpPacket     Smsg_hp   ;

extern ReceivePacket    Rmsg;
extern Navigation       navigation;

#endif
