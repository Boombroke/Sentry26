#!/usr/bin/env python3
# Auto-generated from protocol.yaml — DO NOT EDIT

import struct
from dataclasses import dataclass

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


HEADER_IMU = 0xA1
HEADER_STATUS = 0xA2
HEADER_HP = 0xA3
HEADER_NAV = 0xB5

@dataclass
class ImuPacket:
    pitch: float = 0.0
    yaw: float = 0.0


ImuPacket.HEADER = 0xA1
ImuPacket.STRUCT_FORMAT = '<Bff'
ImuPacket.TOTAL_SIZE = 11
ImuPacket.FREQUENCY_HZ = 1000
ImuPacket.DIRECTION = 'stm32_to_ros'
ImuPacket.FIELDS_META = [
    {'name': 'pitch', 'type': 'float', 'unit': 'rad', 'desc': '云台 pitch'},
    {'name': 'yaw', 'type': 'float', 'unit': 'rad', 'desc': '云台 yaw'},
]

@dataclass
class StatusPacket:
    game_progress: int = 0
    stage_remain_time: int = 0
    current_hp: int = 0
    projectile_allowance_17mm: int = 0
    team_colour: int = 0
    rfid_base: int = 0


StatusPacket.HEADER = 0xA2
StatusPacket.STRUCT_FORMAT = '<BBHHHBB'
StatusPacket.TOTAL_SIZE = 12
StatusPacket.FREQUENCY_HZ = 10
StatusPacket.DIRECTION = 'stm32_to_ros'
StatusPacket.FIELDS_META = [
    {'name': 'game_progress', 'type': 'uint8', 'desc': '游戏阶段 0-未开始 1-准备 2-自检 3-倒计时 4-比赛中 5-结算'},
    {'name': 'stage_remain_time', 'type': 'uint16', 'unit': 's', 'desc': '当前阶段剩余时间'},
    {'name': 'current_hp', 'type': 'uint16', 'desc': '机器人当前血量'},
    {'name': 'projectile_allowance_17mm', 'type': 'uint16', 'desc': '17mm弹丸剩余发射次数'},
    {'name': 'team_colour', 'type': 'uint8', 'desc': '1=red 0=blue'},
    {'name': 'rfid_base', 'type': 'uint8', 'desc': '己方基地增益点'},
]

@dataclass
class HpPacket:
    ally_1_robot_hp: int = 0
    ally_2_robot_hp: int = 0
    ally_3_robot_hp: int = 0
    ally_4_robot_hp: int = 0
    ally_7_robot_hp: int = 0
    ally_outpost_hp: int = 0
    ally_base_hp: int = 0


HpPacket.HEADER = 0xA3
HpPacket.STRUCT_FORMAT = '<BHHHHHHH'
HpPacket.TOTAL_SIZE = 17
HpPacket.FREQUENCY_HZ = 2
HpPacket.DIRECTION = 'stm32_to_ros'
HpPacket.FIELDS_META = [
    {'name': 'ally_1_robot_hp', 'type': 'uint16', 'desc': '己方1号英雄血量'},
    {'name': 'ally_2_robot_hp', 'type': 'uint16', 'desc': '己方2号工程血量'},
    {'name': 'ally_3_robot_hp', 'type': 'uint16', 'desc': '己方3号步兵血量'},
    {'name': 'ally_4_robot_hp', 'type': 'uint16', 'desc': '己方4号步兵血量'},
    {'name': 'ally_7_robot_hp', 'type': 'uint16', 'desc': '己方7号哨兵血量'},
    {'name': 'ally_outpost_hp', 'type': 'uint16', 'desc': '己方前哨站血量'},
    {'name': 'ally_base_hp', 'type': 'uint16', 'desc': '己方基地血量'},
]

@dataclass
class NavPacket:
    vel_x: float = 0.0
    vel_y: float = 0.0
    vel_w: float = 0.0


NavPacket.HEADER = 0xB5
NavPacket.STRUCT_FORMAT = '<Bfff'
NavPacket.TOTAL_SIZE = 15
NavPacket.FREQUENCY_HZ = 20
NavPacket.DIRECTION = 'ros_to_stm32'
NavPacket.FIELDS_META = [
    {'name': 'vel_x', 'type': 'float', 'unit': 'm/s', 'desc': '底盘x方向线速度'},
    {'name': 'vel_y', 'type': 'float', 'unit': 'm/s', 'desc': '底盘y方向线速度'},
    {'name': 'vel_w', 'type': 'float', 'unit': 'rad/s', 'desc': '底盘角速度'},
]

PACKETS = {
    0xA1: ImuPacket,
    0xA2: StatusPacket,
    0xA3: HpPacket,
    0xB5: NavPacket,
}

STM32_TO_ROS_PACKETS = [
    ImuPacket,
    StatusPacket,
    HpPacket,
]

ROS_TO_STM32_PACKETS = [
    NavPacket,
]


def pack_with_crc(packet) -> bytes:
    values = [packet.HEADER]
    for meta in packet.FIELDS_META:
        values.append(getattr(packet, meta['name']))
    body = struct.pack(packet.STRUCT_FORMAT, *values)
    return body + struct.pack('<H', crc16(body))


def unpack_packet(data: bytes):
    if len(data) < 3:
        return None
    header = data[0]
    pkt_class = PACKETS.get(header)
    if pkt_class is None:
        return None
    if len(data) < pkt_class.TOTAL_SIZE:
        return None

    body = data[:pkt_class.TOTAL_SIZE - 2]
    recv_crc = struct.unpack('<H', data[pkt_class.TOTAL_SIZE - 2:pkt_class.TOTAL_SIZE])[0]
    crc_ok = crc16(body) == recv_crc

    unpacked = struct.unpack(pkt_class.STRUCT_FORMAT, body)
    pkt = pkt_class()
    for i, meta in enumerate(pkt_class.FIELDS_META):
        setattr(pkt, meta['name'], unpacked[i + 1])
    return pkt, crc_ok


def packet_size_for_header(header: int) -> int:
    pkt_class = PACKETS.get(header)
    return pkt_class.TOTAL_SIZE if pkt_class else 0
