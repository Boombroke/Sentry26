#!/usr/bin/env python3
# 持续 publisher: 同时发 GameStatus + RobotStatus, 5Hz, 永远跑直到 SIGINT.
# 用于真仿真验证 (referee 必须持续 publish, 不能像 inv_smoke mock 那样一过性).
#
# Usage:
#   ROS_NAMESPACE=red_standard_robot1 python3 tests/pub_referee_persist.py \
#       --progress 4 --hp 300 --ammo 100 --rate-hz 5
#
import argparse
import signal
import sys
import time

import rclpy
from rclpy.node import Node

from rm_interfaces.msg import GameStatus, RobotStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--progress", type=int, default=4)
    parser.add_argument("--remain", type=int, default=200)
    parser.add_argument("--hp", type=int, default=300)
    parser.add_argument("--ammo", type=int, default=100)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--game-topic", default="referee/game_status")
    parser.add_argument("--robot-topic", default="referee/robot_status")
    parser.add_argument("--namespace", default="")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Node("test_referee_persist_pub", namespace=args.namespace)

    g_pub = node.create_publisher(GameStatus, args.game_topic, 10)
    r_pub = node.create_publisher(RobotStatus, args.robot_topic, 10)

    g_msg = GameStatus()
    g_msg.game_progress = args.progress
    g_msg.stage_remain_time = args.remain
    g_msg.behavior_state = 0

    r_msg = RobotStatus()
    r_msg.current_hp = args.hp
    r_msg.projectile_allowance_17mm = args.ammo
    r_msg.maximum_hp = 400

    period = 1.0 / args.rate_hz
    print(
        f"[PUB] start game_topic={args.game_topic} robot_topic={args.robot_topic} "
        f"progress={args.progress} hp={args.hp} ammo={args.ammo}",
        flush=True,
    )

    def _shutdown(*_):
        print("[PUB] SIGINT", flush=True)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    while rclpy.ok():
        # Allow runtime mutation via flag file
        try:
            with open("/tmp/sentry_pub_state.txt", "r", encoding="utf-8") as f:
                line = f.read().strip()
                if line:
                    parts = dict(kv.split("=", 1) for kv in line.split() if "=" in kv)
                    if "hp" in parts:
                        r_msg.current_hp = int(parts["hp"])
                    if "ammo" in parts:
                        r_msg.projectile_allowance_17mm = int(parts["ammo"])
                    if "progress" in parts:
                        g_msg.game_progress = int(parts["progress"])
        except FileNotFoundError:
            pass
        except Exception as e:
            print(f"[PUB] state file parse error: {e}", flush=True)

        g_pub.publish(g_msg)
        r_pub.publish(r_msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


if __name__ == "__main__":
    main()
