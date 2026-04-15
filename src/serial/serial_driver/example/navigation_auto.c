#include "navigation_auto.h"
#include "cmsis_os.h"
#include "CRCs.h"
#include "usbd_cdc_if.h"
#include "referee.h"
#include "ins_Task.h"
#include "ModeControl.h"

static uint8_t usb_tx_buf[APP_TX_DATA_SIZE];
static uint8_t usb_rx_buf[APP_RX_DATA_SIZE];

SendImuPacket    Smsg_imu;
SendStatusPacket Smsg_status;
SendHpPacket     Smsg_hp;
ReceivePacket    Rmsg;
Navigation       navigation;

static uint32_t imu_counter    = 0;
static uint32_t status_counter = 0;
static uint32_t hp_counter     = 0;

void Append_CRC16_Check_Sum_Buf(uint8_t *buf, uint32_t len)
{
  if (buf == NULL || len <= 2) return;
  uint16_t crc = CRC16_Calculate(buf, len - 2);
  buf[len - 2] = (uint8_t)(crc & 0xFF);
  buf[len - 1] = (uint8_t)((crc >> 8) & 0xFF);
}

void Navigation_Init(void)
{
  memset(&Smsg_imu, 0, sizeof(Smsg_imu));
  memset(&Smsg_status, 0, sizeof(Smsg_status));
  memset(&Smsg_hp, 0, sizeof(Smsg_hp));
  memset(&Rmsg, 0, sizeof(Rmsg));
  memset(&navigation, 0, sizeof(navigation));

  Smsg_imu.header    = FH_TX_IMU;
  Smsg_status.header = FH_TX_STATUS;
  Smsg_hp.header     = FH_TX_HP;
}

void send_ImuPacket(void)
{
  Smsg_imu.pitch = -INS.Pitch / 57.3f;
  Smsg_imu.yaw   =  INS.Yaw   / 57.3f;

  Append_CRC16_Check_Sum_Buf((uint8_t *)&Smsg_imu, sizeof(Smsg_imu));
  memcpy(usb_tx_buf, &Smsg_imu, sizeof(Smsg_imu));
  CDC_Transmit_FS(usb_tx_buf, sizeof(Smsg_imu));
}

void send_StatusPacket(void)
{
  uint8_t team_colour = (Robot_Status.robot_id < 100) ? 1 : 0;

  Smsg_status.game_progress             = Game_Status.game_progress;
  Smsg_status.stage_remain_time         = Game_Status.stage_remain_time;
  Smsg_status.current_hp                = Robot_Status.current_HP;
  Smsg_status.projectile_allowance_17mm = Projectile_Allowance.projectile_allowance_17mm;
  Smsg_status.team_colour               = team_colour;
  Smsg_status.rfid_base                 = (RFID_Status.rfid_status == 1) ? 1 : 0;

  Append_CRC16_Check_Sum_Buf((uint8_t *)&Smsg_status, sizeof(Smsg_status));
  memcpy(usb_tx_buf, &Smsg_status, sizeof(Smsg_status));
  CDC_Transmit_FS(usb_tx_buf, sizeof(Smsg_status));
}

void send_HpPacket(void)
{
  Smsg_hp.ally_1_robot_HP = Game_Robot_HP.ally_1_robot_HP;
  Smsg_hp.ally_2_robot_HP = Game_Robot_HP.ally_2_robot_HP;
  Smsg_hp.ally_3_robot_HP = Game_Robot_HP.ally_3_robot_HP;
  Smsg_hp.ally_4_robot_HP = Game_Robot_HP.ally_4_robot_HP;
  Smsg_hp.ally_7_robot_HP = Game_Robot_HP.ally_7_robot_HP;
  Smsg_hp.ally_outpost_HP = Game_Robot_HP.ally_outpost_HP;
  Smsg_hp.ally_base_HP    = Game_Robot_HP.ally_base_HP;

  Append_CRC16_Check_Sum_Buf((uint8_t *)&Smsg_hp, sizeof(Smsg_hp));
  memcpy(usb_tx_buf, &Smsg_hp, sizeof(Smsg_hp));
  CDC_Transmit_FS(usb_tx_buf, sizeof(Smsg_hp));
}

void receive_Navigation(void)
{
  CDC_Receive_FS(usb_rx_buf, sizeof(ReceivePacket));

  if (usb_rx_buf[0] == FH_RX) {
    if (CRC16_Verify(usb_rx_buf, sizeof(ReceivePacket))) {
      memcpy(&Rmsg, usb_rx_buf, sizeof(ReceivePacket));
      Navigation_Parm(&navigation);
    }
  }
}

void Navigation_Parm(Navigation *nav)
{
  nav->lx    = Rmsg.lx;
  nav->ly    = Rmsg.ly;
  nav->vel_w = Rmsg.vel_w;
}

void Navigation_Task(void)
{
  receive_Navigation();

  imu_counter++;
  status_counter++;
  hp_counter++;

  if (imu_counter >= SEND_IMU_INTERVAL) {
    send_ImuPacket();
    imu_counter = 0;
  }

  if (status_counter >= SEND_STATUS_INTERVAL) {
    send_StatusPacket();
    status_counter = 0;
  }

  if (hp_counter >= SEND_HP_INTERVAL) {
    send_HpPacket();
    hp_counter = 0;
  }
}

