#!/usr/bin/env python3
# Copyright 2026 Boombroke
# Licensed under the Apache License, Version 2.0
"""Auto rosbag recorder for RoboMaster sentry, triggered by referee game_progress."""

import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm_interfaces.msg import GameStatus


class MatchRecorder(Node):
    """State machine that spawns `ros2 bag record` while game_progress matches target."""

    def __init__(self):
        super().__init__('match_recorder')

        # --- Parameters ---
        self.declare_parameter('record_dir', 'logs/match-bags')
        self.declare_parameter('sortie_prefix', 'sortie')
        self.declare_parameter('slice_seconds', 60)
        self.declare_parameter('storage_id', 'mcap')
        self.declare_parameter('debounce_count', 3)
        self.declare_parameter('max_session_seconds', 480)
        self.declare_parameter('target_progress', 4)
        self.declare_parameter('topics_file', '')
        # Empty default list cannot be auto-typed by rclpy; allow dynamic typing
        # so the YAML override (string list) is accepted.
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('extra_topics', [], dyn)
        self.declare_parameter('exclude_topics', [], dyn)

        self.record_dir = self.get_parameter('record_dir').value
        self.sortie_prefix = self.get_parameter('sortie_prefix').value
        self.slice_seconds = int(self.get_parameter('slice_seconds').value)
        self.storage_id = self.get_parameter('storage_id').value
        self.debounce_count = int(self.get_parameter('debounce_count').value)
        self.max_session_seconds = int(
            self.get_parameter('max_session_seconds').value)
        self.target_progress = int(self.get_parameter('target_progress').value)

        # --- State ---
        self.state = 'IDLE'
        self.consec_active = 0
        self.consec_inactive = 0
        self.proc = None
        self.session_start = 0.0
        self.session_dir = None

        # --- IO ---
        self.sub = self.create_subscription(
            GameStatus,
            '/referee/game_status',
            self._on_game_status,
            qos_profile_sensor_data,
        )
        self.watchdog_timer = self.create_timer(0.5, self._watchdog)

        self.get_logger().info(
            f'[recorder] ready. target_progress={self.target_progress}, '
            f'debounce_count={self.debounce_count}, '
            f'record_dir={self.record_dir}, storage_id={self.storage_id}'
        )

    # ------------------------------------------------------------------ #
    # Topic resolution                                                   #
    # ------------------------------------------------------------------ #
    def _resolve_topics_file(self):
        p = self.get_parameter('topics_file').value
        if p:
            return Path(p)
        return Path(get_package_share_directory('sentry_match_recorder')) / 'topics.yaml'

    def _build_topics_list(self):
        topics_file = self._resolve_topics_file()
        base_topics = []
        try:
            with open(topics_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            base_topics = list(data.get('topics', []) or [])
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f'[recorder] failed to load topics file {topics_file}: {e}')
            base_topics = []

        extra = list(self.get_parameter('extra_topics').value or [])
        exclude = set(self.get_parameter('exclude_topics').value or [])

        merged = base_topics + extra
        result = []
        seen = set()
        for t in merged:
            if not t or t in exclude or t in seen:
                continue
            seen.add(t)
            result.append(t)
        return result

    # ------------------------------------------------------------------ #
    # Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def _on_game_status(self, msg: GameStatus):
        is_active = (msg.game_progress == self.target_progress)
        if is_active:
            self.consec_active += 1
            self.consec_inactive = 0
        else:
            self.consec_inactive += 1
            self.consec_active = 0

        if self.state == 'IDLE' and self.consec_active >= self.debounce_count:
            self._start_recording()
        elif self.state == 'RECORDING' and self.consec_inactive >= self.debounce_count:
            self._stop_recording(reason=f'game_progress != {self.target_progress}')

    def _watchdog(self):
        if self.state != 'RECORDING' or self.proc is None:
            return
        rc = self.proc.poll()
        if rc is not None:
            self.get_logger().error(
                f'[recorder] bag record exited unexpectedly (rc={rc})')
            self.proc = None
            self.state = 'IDLE'
            self.consec_active = 0
            self.consec_inactive = 0
            return

        if time.time() - self.session_start > self.max_session_seconds:
            self._stop_recording(reason='max_session_seconds')

    # ------------------------------------------------------------------ #
    # Recording control                                                  #
    # ------------------------------------------------------------------ #
    def _start_recording(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        session_dir = Path(self.record_dir) / f'{self.sortie_prefix}_{ts}'
        session_dir.parent.mkdir(parents=True, exist_ok=True)

        topics_list = self._build_topics_list()
        if not topics_list:
            self.get_logger().error(
                '[recorder] empty topics list; aborting start')
            self.consec_active = 0
            return

        cmd = [
            'ros2', 'bag', 'record',
            '-o', str(session_dir),
            '--storage', self.storage_id,
            '--max-bag-duration', str(self.slice_seconds),
            *topics_list,
        ]

        try:
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'[recorder] failed to spawn ros2 bag: {e}')
            self.proc = None
            self.consec_active = 0
            return

        self.session_dir = session_dir
        self.session_start = time.time()
        self.state = 'RECORDING'
        self.consec_inactive = 0
        self.get_logger().info(f'[recorder] sortie started -> {session_dir}')

    def _stop_recording(self, reason: str = ''):
        if self.proc is None:
            self.state = 'IDLE'
            self.consec_active = 0
            self.consec_inactive = 0
            return

        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'[recorder] SIGINT failed: {e}')

        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                '[recorder] bag record did not exit in 10s, escalating to SIGTERM')
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'[recorder] SIGTERM failed: {e}')
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    '[recorder] bag record still alive after SIGTERM')

        self.get_logger().info(
            f'[recorder] sortie stopped (reason: {reason}) dir={self.session_dir}')
        self.proc = None
        self.session_dir = None
        self.session_start = 0.0
        self.state = 'IDLE'
        self.consec_active = 0
        self.consec_inactive = 0

    # ------------------------------------------------------------------ #
    # Lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def destroy_node(self):
        if self.state == 'RECORDING' and self.proc is not None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except Exception:  # noqa: BLE001
                pass
            try:
                self.proc.wait(timeout=5)
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MatchRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.state == 'RECORDING' and node.proc is not None:
            try:
                os.killpg(os.getpgid(node.proc.pid), signal.SIGINT)
                node.proc.wait(timeout=5)
            except Exception:  # noqa: BLE001
                pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
