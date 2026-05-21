#!/usr/bin/env python3
# 测试用 referee publisher: 直接用 rclpy 发, 避开 ros2 topic pub CLI
# 在某些 jazzy 环境对带 # 中文注释的 .msg 解析失败.
#
# Usage:
#   pub_referee.py game --progress 4 --remain 200
#   pub_referee.py robot --hp 300 --ammo 100

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from rm_interfaces.msg import GameStatus, RobotStatus


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("game")
    g.add_argument("--progress", type=int, default=4)
    g.add_argument("--remain", type=int, default=200)
    g.add_argument("--rate-hz", type=float, default=5.0)
    g.add_argument("--count", type=int, default=5)

    r = sub.add_parser("robot")
    r.add_argument("--hp", type=int, default=300)
    r.add_argument("--ammo", type=int, default=100)
    r.add_argument("--rate-hz", type=float, default=5.0)
    r.add_argument("--count", type=int, default=5)

    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Node("test_referee_pub")

    # 5 条 (10Hz) 已经够 BT 命中, 用默认 QoS reliable+volatile 与 server subscriber 匹配.
    if args.cmd == "game":
        pub = node.create_publisher(GameStatus, "/referee/game_status", 10)
        msg = GameStatus()
        msg.game_progress = args.progress
        msg.stage_remain_time = args.remain
        msg.behavior_state = 0
    else:
        pub = node.create_publisher(RobotStatus, "/referee/robot_status", 10)
        msg = RobotStatus()
        msg.current_hp = args.hp
        msg.projectile_allowance_17mm = args.ammo
        msg.maximum_hp = 400

    # 等 publisher 与已存在的 subscriber 完成 discovery
    deadline = time.time() + 1.5
    while time.time() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)

    period = 1.0 / args.rate_hz
    for _ in range(args.count):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
