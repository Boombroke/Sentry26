#!/usr/bin/env python3
# Mock NavigateToPose action server, only for INV/regression tests.
#
# Usage:
#   ./tests/mock_navigate_to_pose_server.py [--result {succeed,abort}] \
#                                           [--duration 1.0] \
#                                           [--node-name mock_nav]
#
# 行为:
#   收到 goal -> 等 duration 秒 -> 按 result 返回 succeed/abort
#   ros2 topic /goal_pose 不被本节点处理 (BT 也不再发该 topic)

import argparse
import signal
import sys
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class MockNavigateToPoseServer(Node):
    def __init__(self, result: str, duration: float, node_name: str):
        super().__init__(node_name)
        self._result = result
        self._duration = duration
        self._action_server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self._execute_callback
        )
        self.get_logger().info(
            f"[MOCK] NavigateToPose server up: result={result} duration={duration}s"
        )

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        pose = goal_handle.request.pose.pose.position
        self.get_logger().info(
            f"[MOCK] received goal x={pose.x:.2f} y={pose.y:.2f}"
        )
        # Simulate navigation delay; abort on cancel request from BT (preempt)
        deadline = time.time() + self._duration
        while time.time() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("[MOCK] returning canceled")
                return NavigateToPose.Result()
            time.sleep(0.05)

        result = NavigateToPose.Result()
        if self._result == "succeed":
            goal_handle.succeed()
            self.get_logger().info("[MOCK] returning succeeded")
        else:
            goal_handle.abort()
            self.get_logger().info("[MOCK] returning aborted")
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", choices=["succeed", "abort"], default="succeed")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--node-name", default="mock_nav")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MockNavigateToPoseServer(args.result, args.duration, args.node_name)

    def _shutdown(_signum, _frame):
        node.get_logger().info("[MOCK] SIGINT, shutting down")
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
