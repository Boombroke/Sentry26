#!/usr/bin/env python3
# AUTO-GENERATED from protocol.yaml — DO NOT EDIT
# Run `python3 generate.py` after modifying protocol.yaml.
#
# Frame format (v3.0):
#   [HEADER 1B] [LEN 1B] [PAYLOAD N B] [CRC16 2B]

import struct
from dataclasses import dataclass, field
from typing import Optional, Tuple

CRC16_INIT = 0xFFFF
W_CRC_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A, 0xBED3,
    0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876, 0x2102, 0x308B, 0x0210, 0x1399,
    0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50,
    0xFBEF, 0xEA66, 0xD8FD, 0xC974, 0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3, 0x5285, 0x430C, 0x7197, 0x601E,
    0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5,
    0xA96A, 0xB8E3, 0x8A78, 0x9BF1, 0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70, 0x8408, 0x9581, 0xA71A, 0xB693,
    0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948, 0x3BD3, 0x2A5A,
    0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E, 0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD, 0xB58B, 0xA402, 0x9699, 0x8710,
    0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF,
    0x0C60, 0x1DE9, 0x2F72, 0x3EFB, 0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A, 0xE70E, 0xF687, 0xC41C, 0xD595,
    0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C,
    0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
]


def crc16(data: bytes) -> int:
    crc = CRC16_INIT
    for b in data:
        crc = (crc >> 8) ^ W_CRC_TABLE[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


# ----------------------------------------------------------------------------
# Header constants
# ----------------------------------------------------------------------------
HEADER_IMU = 0x51  # stm32_to_ros, 200Hz
HEADER_CHASSIS_FEEDBACK = 0x52  # stm32_to_ros, 20Hz
HEADER_REFEREE = 0x53  # stm32_to_ros, 1Hz
HEADER_NAV_CMD = 0xA2  # ros_to_stm32, 20Hz
HEADER_HEARTBEAT = 0xA3  # ros_to_stm32, 1Hz


# ----------------------------------------------------------------------------
# Per-packet dataclasses
# ----------------------------------------------------------------------------

@dataclass
class ImuPacket:
    """stm32_to_ros, 200Hz, frame=22B"""
    gimbal_pitch: float = 0.0  # 云台 pitch（绕 y） [rad]

    gimbal_yaw: float = 0.0  # 云台 yaw（绕 z） [rad]

    chassis_pitch: float = 0.0  # 车体 pitch（绕 y），轮足姿态 [rad]

    chassis_yaw: float = 0.0  # 车体 yaw（绕 z） [rad]

    mcu_timestamp_ms: int = 0  # MCU 时间戳低 16 位，便于估算时延 [ms]



ImuPacket.HEADER          = 0x51
ImuPacket.PAYLOAD_SIZE    = 18
ImuPacket.FRAME_SIZE      = 22
ImuPacket.STRUCT_NO_CRC   = '<BBffffH'  # header + len + payload
ImuPacket.STRUCT_FULL     = '<BBffffHH'    # full frame incl CRC
ImuPacket.FREQUENCY_HZ    = 200
ImuPacket.DIRECTION       = 'stm32_to_ros'
ImuPacket.FIELDS_META     = [
    {'name': 'gimbal_pitch', 'type': 'float', 'unit': 'rad', 'desc': '云台 pitch（绕 y）'},
    {'name': 'gimbal_yaw', 'type': 'float', 'unit': 'rad', 'desc': '云台 yaw（绕 z）'},
    {'name': 'chassis_pitch', 'type': 'float', 'unit': 'rad', 'desc': '车体 pitch（绕 y），轮足姿态'},
    {'name': 'chassis_yaw', 'type': 'float', 'unit': 'rad', 'desc': '车体 yaw（绕 z）'},
    {'name': 'mcu_timestamp_ms', 'type': 'uint16', 'unit': 'ms', 'desc': 'MCU 时间戳低 16 位，便于估算时延'},
]


@dataclass
class ChassisFeedbackPacket:
    """stm32_to_ros, 20Hz, frame=14B"""
    current_hp: int = 0  # 本机当前血量 [hp]

    projectile_allowance_17mm: int = 0  # 17mm 弹丸剩余发射次数 [count]

    chassis_power: float = 0.0  # 底盘实时功率 [W]

    chassis_mode: int = 0  # 电控实际模式 0=normal 1=spin_low 2=spin_high 3=estop

    reserved: int = 0  # 字节对齐占位，保留



ChassisFeedbackPacket.HEADER          = 0x52
ChassisFeedbackPacket.PAYLOAD_SIZE    = 10
ChassisFeedbackPacket.FRAME_SIZE      = 14
ChassisFeedbackPacket.STRUCT_NO_CRC   = '<BBHHfBB'  # header + len + payload
ChassisFeedbackPacket.STRUCT_FULL     = '<BBHHfBBH'    # full frame incl CRC
ChassisFeedbackPacket.FREQUENCY_HZ    = 20
ChassisFeedbackPacket.DIRECTION       = 'stm32_to_ros'
ChassisFeedbackPacket.FIELDS_META     = [
    {'name': 'current_hp', 'type': 'uint16', 'unit': 'hp', 'desc': '本机当前血量'},
    {'name': 'projectile_allowance_17mm', 'type': 'uint16', 'unit': 'count', 'desc': '17mm 弹丸剩余发射次数'},
    {'name': 'chassis_power', 'type': 'float', 'unit': 'W', 'desc': '底盘实时功率'},
    {'name': 'chassis_mode', 'type': 'uint8', 'desc': '电控实际模式 0=normal 1=spin_low 2=spin_high 3=estop'},
    {'name': 'reserved', 'type': 'uint8', 'desc': '字节对齐占位，保留'},
]


@dataclass
class RefereePacket:
    """stm32_to_ros, 1Hz, frame=27B"""
    game_progress: int = 0  # 0=未开始 1=准备 2=自检 3=5s倒计时 4=比赛中 5=结算

    stage_remain_time: int = 0  # 当前阶段剩余时间 [s]

    team_colour: int = 0  # 1=红方 0=蓝方

    rfid_base: int = 0  # 己方基地增益点 RFID（1=触发）

    ally_1_robot_hp: int = 0  # 己方 1 号英雄血量 [hp]

    ally_2_robot_hp: int = 0  # 己方 2 号工程血量 [hp]

    ally_3_robot_hp: int = 0  # 己方 3 号步兵血量 [hp]

    ally_4_robot_hp: int = 0  # 己方 4 号步兵血量 [hp]

    ally_7_robot_hp: int = 0  # 己方 7 号哨兵血量 [hp]

    ally_outpost_hp: int = 0  # 己方前哨站血量 [hp]

    ally_base_hp: int = 0  # 己方基地血量 [hp]

    event_data: int = 0  # 事件数据 bitfield，1=己方增益点 2=己方堡垒被占



RefereePacket.HEADER          = 0x53
RefereePacket.PAYLOAD_SIZE    = 23
RefereePacket.FRAME_SIZE      = 27
RefereePacket.STRUCT_NO_CRC   = '<BBBHBBHHHHHHHI'  # header + len + payload
RefereePacket.STRUCT_FULL     = '<BBBHBBHHHHHHHIH'    # full frame incl CRC
RefereePacket.FREQUENCY_HZ    = 1
RefereePacket.DIRECTION       = 'stm32_to_ros'
RefereePacket.FIELDS_META     = [
    {'name': 'game_progress', 'type': 'uint8', 'desc': '0=未开始 1=准备 2=自检 3=5s倒计时 4=比赛中 5=结算'},
    {'name': 'stage_remain_time', 'type': 'uint16', 'unit': 's', 'desc': '当前阶段剩余时间'},
    {'name': 'team_colour', 'type': 'uint8', 'desc': '1=红方 0=蓝方'},
    {'name': 'rfid_base', 'type': 'uint8', 'desc': '己方基地增益点 RFID（1=触发）'},
    {'name': 'ally_1_robot_hp', 'type': 'uint16', 'unit': 'hp', 'desc': '己方 1 号英雄血量'},
    {'name': 'ally_2_robot_hp', 'type': 'uint16', 'unit': 'hp', 'desc': '己方 2 号工程血量'},
    {'name': 'ally_3_robot_hp', 'type': 'uint16', 'unit': 'hp', 'desc': '己方 3 号步兵血量'},
    {'name': 'ally_4_robot_hp', 'type': 'uint16', 'unit': 'hp', 'desc': '己方 4 号步兵血量'},
    {'name': 'ally_7_robot_hp', 'type': 'uint16', 'unit': 'hp', 'desc': '己方 7 号哨兵血量'},
    {'name': 'ally_outpost_hp', 'type': 'uint16', 'unit': 'hp', 'desc': '己方前哨站血量'},
    {'name': 'ally_base_hp', 'type': 'uint16', 'unit': 'hp', 'desc': '己方基地血量'},
    {'name': 'event_data', 'type': 'uint32', 'desc': '事件数据 bitfield，1=己方增益点 2=己方堡垒被占'},
]


@dataclass
class NavCmdPacket:
    """ros_to_stm32, 20Hz, frame=18B"""
    lx: float = 0.0  # 底盘 body 系前向线速度 [m/s]

    ly: float = 0.0  # 底盘 body 系侧向线速度（差速底盘锁 0） [m/s]

    az: float = 0.0  # 底盘角速度 / 自旋速度 [rad/s]

    mode: int = 0  # 0=normal 1=spin_low 2=spin_high 3=estop

    reserved: int = 0  # 字节对齐占位，保留



NavCmdPacket.HEADER          = 0xA2
NavCmdPacket.PAYLOAD_SIZE    = 14
NavCmdPacket.FRAME_SIZE      = 18
NavCmdPacket.STRUCT_NO_CRC   = '<BBfffBB'  # header + len + payload
NavCmdPacket.STRUCT_FULL     = '<BBfffBBH'    # full frame incl CRC
NavCmdPacket.FREQUENCY_HZ    = 20
NavCmdPacket.DIRECTION       = 'ros_to_stm32'
NavCmdPacket.FIELDS_META     = [
    {'name': 'lx', 'type': 'float', 'unit': 'm/s', 'desc': '底盘 body 系前向线速度'},
    {'name': 'ly', 'type': 'float', 'unit': 'm/s', 'desc': '底盘 body 系侧向线速度（差速底盘锁 0）'},
    {'name': 'az', 'type': 'float', 'unit': 'rad/s', 'desc': '底盘角速度 / 自旋速度'},
    {'name': 'mode', 'type': 'uint8', 'desc': '0=normal 1=spin_low 2=spin_high 3=estop'},
    {'name': 'reserved', 'type': 'uint8', 'desc': '字节对齐占位，保留'},
]


@dataclass
class HeartbeatPacket:
    """ros_to_stm32, 1Hz, frame=6B"""
    ros_state: int = 0  # 0=init 1=ready 2=running 3=fault

    reserved: int = 0  # 字节对齐占位，保留



HeartbeatPacket.HEADER          = 0xA3
HeartbeatPacket.PAYLOAD_SIZE    = 2
HeartbeatPacket.FRAME_SIZE      = 6
HeartbeatPacket.STRUCT_NO_CRC   = '<BBBB'  # header + len + payload
HeartbeatPacket.STRUCT_FULL     = '<BBBBH'    # full frame incl CRC
HeartbeatPacket.FREQUENCY_HZ    = 1
HeartbeatPacket.DIRECTION       = 'ros_to_stm32'
HeartbeatPacket.FIELDS_META     = [
    {'name': 'ros_state', 'type': 'uint8', 'desc': '0=init 1=ready 2=running 3=fault'},
    {'name': 'reserved', 'type': 'uint8', 'desc': '字节对齐占位，保留'},
]



# ----------------------------------------------------------------------------
# Header → class lookup
# ----------------------------------------------------------------------------
PACKETS = {
    0x51: ImuPacket,
    0x52: ChassisFeedbackPacket,
    0x53: RefereePacket,
    0xA2: NavCmdPacket,
    0xA3: HeartbeatPacket,
}

STM32_TO_ROS_PACKETS = [
    ImuPacket,
    ChassisFeedbackPacket,
    RefereePacket,
]

ROS_TO_STM32_PACKETS = [
    NavCmdPacket,
    HeartbeatPacket,
]


def pack_with_crc(packet) -> bytes:
    """Build a complete frame: [HEADER][LEN][PAYLOAD][CRC16]."""
    values = [packet.HEADER, packet.PAYLOAD_SIZE]
    for meta in packet.FIELDS_META:
        values.append(getattr(packet, meta['name']))
    body = struct.pack(packet.STRUCT_NO_CRC, *values)
    return body + struct.pack('<H', crc16(body))


def unpack_packet(data: bytes) -> Optional[Tuple[object, bool]]:
    """Decode a frame; returns (packet_instance, crc_ok) or None on bad/incomplete input."""
    if len(data) < 4:
        return None
    header = data[0]
    plen = data[1]
    pkt_class = PACKETS.get(header)
    if pkt_class is None:
        return None
    if plen != pkt_class.PAYLOAD_SIZE:
        return None
    if len(data) < pkt_class.FRAME_SIZE:
        return None

    body = data[:pkt_class.FRAME_SIZE - 2]
    recv_crc = struct.unpack('<H', data[pkt_class.FRAME_SIZE - 2:pkt_class.FRAME_SIZE])[0]
    crc_ok = crc16(body) == recv_crc

    unpacked = struct.unpack(pkt_class.STRUCT_NO_CRC, body)
    pkt = pkt_class()
    # unpacked = (header, len, *fields)
    for i, meta in enumerate(pkt_class.FIELDS_META):
        setattr(pkt, meta['name'], unpacked[i + 2])
    return pkt, crc_ok


def packet_size_for_header(header: int) -> int:
    pkt_class = PACKETS.get(header)
    return pkt_class.FRAME_SIZE if pkt_class else 0
