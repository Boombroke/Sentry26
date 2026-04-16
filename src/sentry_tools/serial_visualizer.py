#!/usr/bin/env python3
# pyright: basic, reportMissingTypeStubs=false, reportAttributeAccessIssue=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportUnannotatedClassAttribute=false, reportUnusedCallResult=false, reportImplicitStringConcatenation=false, reportImplicitOverride=false, reportMissingParameterType=false

"""Real-time serial/ROS2 data visualizer — pyqtgraph + OpenGL backend.

Performance: ~50 FPS sustained with 4 scrolling plots + dashboard panels.
Replaces the original matplotlib-based version (limited to ~5-10 FPS).
"""

import math
import sys
import threading
import time

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rm_interfaces.msg import GameRobotHP, GameStatus, RobotStatus
from sensor_msgs.msg import JointState

# OpenGL disabled by default — Intel iGPU can only serve one GL context, and
# Gazebo simulation already takes it.  Set VIZ_OPENGL=1 to force GPU rendering
# when running without Gazebo (e.g. real robot).
import os
_use_opengl = os.environ.get('VIZ_OPENGL', '0') == '1'
pg.setConfigOptions(useOpenGL=_use_opengl, antialias=False,
                    foreground='#d0d0d0', background='#1e1e2e')

GAME_PROGRESS_LABELS = [
    '\u672a\u5f00\u59cb', '\u51c6\u5907\u9636\u6bb5', '\u88c1\u5224\u7cfb\u7edf\u81ea\u68c0', '\u4e94\u79d2\u5012\u8ba1\u65f6', '\u6bd4\u8d5b\u4e2d', '\u7ed3\u7b97',
]

WINDOW_SECONDS = 10.0
UI_INTERVAL_MS = 50
MAXLEN = 400
ROBOT_HP_MAX = 600
OUTPOST_HP_MAX = 1500
BASE_HP_MAX = 1500


class RingBuffer:
    __slots__ = ('data', 'maxlen', 'idx', 'count')

    def __init__(self, maxlen=MAXLEN):
        self.data = np.zeros(maxlen, dtype=np.float64)
        self.maxlen = maxlen
        self.idx = 0
        self.count = 0

    def append(self, value):
        self.data[self.idx] = value
        self.idx = (self.idx + 1) % self.maxlen
        if self.count < self.maxlen:
            self.count += 1

    def get_ordered(self):
        if self.count < self.maxlen:
            return self.data[:self.count].copy()
        return np.concatenate([self.data[self.idx:], self.data[:self.idx]])


class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.gimbal_t = RingBuffer(); self.gimbal_pitch = RingBuffer(); self.gimbal_yaw = RingBuffer()
        self.cmd_t = RingBuffer(); self.cmd_vx = RingBuffer(); self.cmd_vy = RingBuffer(); self.cmd_vw = RingBuffer()
        self.odom_t = RingBuffer(); self.odom_vx = RingBuffer(); self.odom_vy = RingBuffer()
        self.odom_vw = RingBuffer(); self.odom_speed = RingBuffer()
        self.err_t = RingBuffer(); self.err_vx = RingBuffer(); self.err_vy = RingBuffer(); self.err_vw = RingBuffer()
        self.final_cmd_t = RingBuffer(); self.final_cmd_vx = RingBuffer()
        self.final_cmd_vy = RingBuffer(); self.final_cmd_vw = RingBuffer()
        self.latest_vx = 0.0; self.latest_vy = 0.0; self.latest_vw = 0.0; self.cmd_count = 0
        self.latest_odom_vx = 0.0; self.latest_odom_vy = 0.0; self.latest_odom_vw = 0.0; self.latest_odom_speed = 0.0
        self.latest_final_vx = 0.0; self.latest_final_vy = 0.0; self.latest_final_vw = 0.0
        self.game_progress = 0; self.stage_remain_time = 0; self.current_hp = 0; self.projectile_allowance = 0
        self.team_hp = {'1\u53f7': 0, '2\u53f7': 0, '3\u53f7': 0, '4\u53f7': 0, '7\u53f7': 0, '\u524d\u54e8': 0, '\u57fa\u5730': 0}
        self.last_rx = {'gimbal': 0.0, 'cmd_vel': 0.0, 'final_cmd': 0.0, 'game': 0.0, 'robot': 0.0, 'hp': 0.0, 'odom': 0.0}


class SerialVisualizerNode(Node):
    def __init__(self, shared):
        super().__init__('serial_visualizer')
        self.shared = shared
        self.create_subscription(JointState, 'serial/gimbal_joint_state', self._on_gimbal, 10)
        self.create_subscription(TwistStamped, 'cmd_vel_nav2_result', self._on_cmd_vel, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_final_cmd_vel, 10)
        self.create_subscription(GameStatus, 'referee/game_status', self._on_game_status, 10)
        self.create_subscription(RobotStatus, 'referee/robot_status', self._on_robot_status, 10)
        self.create_subscription(GameRobotHP, 'referee/all_robot_hp', self._on_all_robot_hp, 10)
        self.create_subscription(Odometry, 'odometry', self._on_odometry, 10)
        self._last_cmd_vx = 0.0; self._last_cmd_vy = 0.0; self._last_cmd_vw = 0.0

    def _on_gimbal(self, msg):
        now = time.monotonic()
        pitch = msg.position[0] if len(msg.position) > 0 else 0.0
        yaw = msg.position[1] if len(msg.position) > 1 else 0.0
        with self.shared.lock:
            self.shared.gimbal_t.append(now); self.shared.gimbal_pitch.append(float(pitch))
            self.shared.gimbal_yaw.append(float(yaw)); self.shared.last_rx['gimbal'] = now

    def _on_cmd_vel(self, msg):
        now = time.monotonic()
        vx, vy, vw = float(msg.twist.linear.x), float(msg.twist.linear.y), float(msg.twist.angular.z)
        self._last_cmd_vx, self._last_cmd_vy, self._last_cmd_vw = vx, vy, vw
        with self.shared.lock:
            self.shared.cmd_t.append(now); self.shared.cmd_vx.append(vx)
            self.shared.cmd_vy.append(vy); self.shared.cmd_vw.append(vw)
            self.shared.latest_vx = vx; self.shared.latest_vy = vy; self.shared.latest_vw = vw
            self.shared.cmd_count += 1; self.shared.last_rx['cmd_vel'] = now

    def _on_final_cmd_vel(self, msg):
        now = time.monotonic()
        with self.shared.lock:
            self.shared.final_cmd_t.append(now)
            self.shared.final_cmd_vx.append(float(msg.linear.x))
            self.shared.final_cmd_vy.append(float(msg.linear.y))
            self.shared.final_cmd_vw.append(float(msg.angular.z))
            self.shared.latest_final_vx = float(msg.linear.x)
            self.shared.latest_final_vy = float(msg.linear.y)
            self.shared.latest_final_vw = float(msg.angular.z)
            self.shared.last_rx['final_cmd'] = now

    def _on_game_status(self, msg):
        now = time.monotonic()
        with self.shared.lock:
            self.shared.game_progress = int(msg.game_progress)
            self.shared.stage_remain_time = int(msg.stage_remain_time)
            self.shared.last_rx['game'] = now

    def _on_robot_status(self, msg):
        now = time.monotonic()
        with self.shared.lock:
            self.shared.current_hp = int(msg.current_hp)
            self.shared.projectile_allowance = int(msg.projectile_allowance_17mm)
            self.shared.last_rx['robot'] = now

    def _on_all_robot_hp(self, msg):
        now = time.monotonic()
        with self.shared.lock:
            self.shared.team_hp['1\u53f7'] = int(msg.ally_1_robot_hp)
            self.shared.team_hp['2\u53f7'] = int(msg.ally_2_robot_hp)
            self.shared.team_hp['3\u53f7'] = int(msg.ally_3_robot_hp)
            self.shared.team_hp['4\u53f7'] = int(msg.ally_4_robot_hp)
            self.shared.team_hp['7\u53f7'] = int(msg.ally_7_robot_hp)
            self.shared.team_hp['\u524d\u54e8'] = int(msg.ally_outpost_hp)
            self.shared.team_hp['\u57fa\u5730'] = int(msg.ally_base_hp)
            self.shared.last_rx['hp'] = now

    def _on_odometry(self, msg):
        now = time.monotonic()
        vx = float(msg.twist.twist.linear.x); vy = float(msg.twist.twist.linear.y)
        vw = float(msg.twist.twist.angular.z); speed = math.sqrt(vx*vx + vy*vy)
        err_vx = self._last_cmd_vx - vx; err_vy = self._last_cmd_vy - vy; err_vw = self._last_cmd_vw - vw
        with self.shared.lock:
            self.shared.odom_t.append(now); self.shared.odom_vx.append(vx)
            self.shared.odom_vy.append(vy); self.shared.odom_vw.append(vw); self.shared.odom_speed.append(speed)
            self.shared.latest_odom_vx = vx; self.shared.latest_odom_vy = vy
            self.shared.latest_odom_vw = vw; self.shared.latest_odom_speed = speed
            self.shared.err_t.append(now); self.shared.err_vx.append(err_vx)
            self.shared.err_vy.append(err_vy); self.shared.err_vw.append(err_vw)
            self.shared.last_rx['odom'] = now


class RosSpinThread(threading.Thread):
    def __init__(self, executor):
        super().__init__(daemon=True); self.executor = executor
    def run(self):
        self.executor.spin()


class PlotCanvas(pg.PlotWidget):
    def __init__(self, title, y_label, curve_configs):
        super().__init__()
        self.setTitle(title, color='#e6e6e6', size='11pt')
        self.setLabel('bottom', '\u65f6\u95f4 (s)', color='#d0d0d0')
        self.setLabel('left', y_label, color='#d0d0d0')
        self.setXRange(-WINDOW_SECONDS, 0, padding=0)
        self.enableAutoRange(axis='y'); self.setAutoVisible(y=True)
        self.showGrid(x=True, y=True, alpha=0.3)
        legend = self.addLegend(offset=(10, 10)); legend.setLabelTextColor('#d0d0d0')
        self.curves = []
        for name, color, style in curve_configs:
            pen = pg.mkPen(color=color, width=1, style=style)
            self.curves.append(self.plot([], [], name=name, pen=pen))

    def update_data(self, datasets):
        for curve, (x, y) in zip(self.curves, datasets):
            curve.setData(x, y)


class DashboardPanel(QtWidgets.QGroupBox):
    def __init__(self, title):
        super().__init__(title)
        self.setStyleSheet(
            'QGroupBox { color: #e5e7eb; border: 1px solid #3a3f57; border-radius: 8px; margin-top: 10px; }'
            'QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }')


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, shared):
        super().__init__()
        self.shared = shared
        self.last_ui_tick = time.monotonic(); self.ui_hz = 0.0
        self.setWindowTitle('Serial Visualizer - Sentry Nav'); self.resize(1500, 1000)
        self._label_colors = {}; self._hp_bar_color = ''
        self._setup_ui(); self._setup_timer()

    def _set_label_color(self, key, label, color):
        if self._label_colors.get(key) != color:
            self._label_colors[key] = color
            label.setStyleSheet(f'font-size: 20px; font-weight: 800; color: {color}; font-family: monospace;')

    def _set_hp_bar_color(self, color):
        if self._hp_bar_color != color:
            self._hp_bar_color = color
            self.hp_bar.setStyleSheet(
                'QProgressBar { border: 1px solid #3f4666; border-radius: 6px; background: #1f2438; color: #e2e8f0; text-align: center; min-height: 22px; font-size: 13px; }'
                f'QProgressBar::chunk {{ border-radius: 6px; background-color: {color}; }}')

    def _setup_ui(self):
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central); root.setContentsMargins(12, 12, 12, 8); root.setSpacing(10)
        body = QtWidgets.QHBoxLayout(); body.setSpacing(10); root.addLayout(body, stretch=1)

        S, D = QtCore.Qt.SolidLine, QtCore.Qt.DashLine
        left = QtWidgets.QWidget(); ll = QtWidgets.QVBoxLayout(left); ll.setSpacing(10); ll.setContentsMargins(0,0,0,0)

        for attr, title, panel_title, y_lbl, cfgs in [
            ('gimbal_plot', 'Pitch / Yaw (\u6700\u8fd110s)', '\u4e91\u53f0\u59ff\u6001', '\u89d2\u5ea6 (rad)', [('pitch','#4f8cff',S),('yaw','#ff9f43',S)]),
            ('vx_plot', 'Vx \u2014 \u547d\u4ee4 vs \u5b9e\u9645 (m/s)', 'Vx \u5bf9\u6bd4', 'm/s', [('cmd','#4f8cff',S),('actual','#4f8cff',D)]),
            ('vy_plot', 'Vy \u2014 \u547d\u4ee4 vs \u5b9e\u9645 (m/s)', 'Vy \u5bf9\u6bd4', 'm/s', [('cmd','#ff9f43',S),('actual','#ff9f43',D)]),
            ('vw_plot', 'Vw \u2014 \u547d\u4ee4 vs \u5b9e\u9645 (rad/s)', 'Vw \u5bf9\u6bd4', 'rad/s', [('cmd','#22c55e',S),('actual','#22c55e',D)]),
        ]:
            box = DashboardPanel(panel_title); lay = QtWidgets.QVBoxLayout(box)
            p = PlotCanvas(title, y_lbl, cfgs); lay.addWidget(p); setattr(self, attr, p)
            ll.addWidget(box, stretch=1)

        right = QtWidgets.QWidget(); rl = QtWidgets.QVBoxLayout(right); rl.setSpacing(10); rl.setContentsMargins(0,0,0,0)

        # Game status
        gb = DashboardPanel('\u6bd4\u8d5b\u72b6\u6001'); gl = QtWidgets.QVBoxLayout(gb)
        self.game_stage_label = QtWidgets.QLabel('\u9636\u6bb5: \u672a\u5f00\u59cb')
        self.game_stage_label.setStyleSheet('font-size: 18px; font-weight: 700; color: #dbeafe;')
        self.game_remain_label = QtWidgets.QLabel('\u5269\u4f59: 0s')
        self.game_remain_label.setStyleSheet('font-size: 22px; font-weight: 800; color: #f8fafc;')
        self.game_progress_bar = QtWidgets.QProgressBar(); self.game_progress_bar.setRange(0, 420)
        self.game_progress_bar.setFormat('%v/%m s')
        self.game_progress_bar.setStyleSheet('QProgressBar { border: 1px solid #3f4666; border-radius: 6px; background: #1f2438; color: #d9e2ff; text-align: center; } QProgressBar::chunk { border-radius: 6px; background-color: #4f8cff; }')
        gl.addWidget(self.game_stage_label); gl.addWidget(self.game_remain_label); gl.addWidget(self.game_progress_bar)

        # Robot status
        rb = DashboardPanel('\u673a\u5668\u4eba\u72b6\u6001'); rbl = QtWidgets.QVBoxLayout(rb)
        hr = QtWidgets.QHBoxLayout()
        ht = QtWidgets.QLabel('\u8840\u91cf:'); ht.setStyleSheet('font-size: 16px; font-weight: 600;')
        self.hp_value_label = QtWidgets.QLabel('0'); self.hp_value_label.setStyleSheet('font-size: 24px; font-weight: 800; color: #f8fafc;')
        hr.addWidget(ht); hr.addStretch(1); hr.addWidget(self.hp_value_label)
        self.hp_bar = QtWidgets.QProgressBar(); self.hp_bar.setRange(0, ROBOT_HP_MAX); self.hp_bar.setFormat('%v/%m')
        self.ammo_label = QtWidgets.QLabel('\u5f39\u91cf: 0 \u53d1'); self.ammo_label.setStyleSheet('font-size: 24px; font-weight: 800; color: #7dd3fc;')
        rbl.addLayout(hr); rbl.addWidget(self.hp_bar); rbl.addWidget(self.ammo_label)

        # Velocity panels helper
        def make_vel_panel(title, labels):
            box = DashboardPanel(title); lay = QtWidgets.QGridLayout(box); lay.setSpacing(6)
            d = {}
            for i, (name, unit) in enumerate(labels):
                n = QtWidgets.QLabel(f'{name}:'); n.setStyleSheet('font-size: 14px; font-weight: 600;')
                v = QtWidgets.QLabel('0.000'); v.setStyleSheet('font-size: 20px; font-weight: 800; color: #f8fafc; font-family: monospace;')
                u = QtWidgets.QLabel(unit); u.setStyleSheet('font-size: 12px; color: #94a3b8;')
                lay.addWidget(n, i, 0); lay.addWidget(v, i, 1); lay.addWidget(u, i, 2); d[name] = v
            return box, lay, d

        txb, txl, self.tx_value_labels = make_vel_panel('\u5bfc\u822a\u547d\u4ee4\u901f\u5ea6 (world \u7cfb)', [('nav_vx','m/s'),('nav_vy','m/s'),('nav_vw','rad/s')])
        self.tx_freq_label = QtWidgets.QLabel('\u9891\u7387: \u2014 Hz'); self.tx_freq_label.setStyleSheet('font-size: 13px; color: #7dd3fc;')
        self.tx_count_label = QtWidgets.QLabel('\u8ba1\u6570: 0'); self.tx_count_label.setStyleSheet('font-size: 13px; color: #94a3b8;')
        tf = QtWidgets.QHBoxLayout(); tf.addWidget(self.tx_freq_label); tf.addStretch(1); tf.addWidget(self.tx_count_label)
        txl.addLayout(tf, 3, 0, 1, 3)

        ab, _, self.actual_value_labels = make_vel_panel('\u5b9e\u9645\u901f\u5ea6 (odometry)', [('act_vx','m/s'),('act_vy','m/s'),('act_vw','rad/s'),('speed','m/s')])
        tb, _, self.track_value_labels = make_vel_panel('\u8ddf\u8e2a\u8bef\u5dee (cmd - actual)', [('\u0394vx','m/s'),('\u0394vy','m/s'),('\u0394vw','rad/s')])
        fb, _, self.final_value_labels = make_vel_panel('\u6700\u7ec8\u4e0b\u53d1\u901f\u5ea6 (body \u7cfb+\u81ea\u65cb)', [('cmd_vx','m/s'),('cmd_vy','m/s'),('cmd_vw','rad/s')])

        # Team HP
        tmb = DashboardPanel('\u5168\u961f\u8840\u91cf'); tml = QtWidgets.QVBoxLayout(tmb); tml.setSpacing(6)
        self.team_bar_widgets = {}
        for label, mx in [('1\u53f7',ROBOT_HP_MAX),('2\u53f7',ROBOT_HP_MAX),('3\u53f7',ROBOT_HP_MAX),('4\u53f7',ROBOT_HP_MAX),('7\u53f7',ROBOT_HP_MAX),('\u524d\u54e8',OUTPOST_HP_MAX),('\u57fa\u5730',BASE_HP_MAX)]:
            row = QtWidgets.QHBoxLayout(); nm = QtWidgets.QLabel(label); nm.setFixedWidth(36)
            bar = QtWidgets.QProgressBar(); bar.setRange(0, mx); bar.setFormat('%v/%m')
            bar.setStyleSheet('QProgressBar { border: 1px solid #3f4666; border-radius: 5px; background: #1f2438; color: #d9e2ff; text-align: center; min-height: 18px; } QProgressBar::chunk { border-radius: 5px; background-color: #34d399; }')
            val = QtWidgets.QLabel('0'); val.setFixedWidth(46); val.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            row.addWidget(nm); row.addWidget(bar, stretch=1); row.addWidget(val); tml.addLayout(row)
            self.team_bar_widgets[label] = (bar, val)

        for w in [gb, rb, txb, ab, tb, fb, tmb]: rl.addWidget(w, stretch=(1 if w is tmb else 0))
        body.addWidget(left, stretch=3); body.addWidget(right, stretch=2)

        self.status_label = QtWidgets.QLabel('Topics: gimbal \u2717 cmd_vel \u2717 odom \u2717 game \u2717 robot \u2717 hp \u2717   20Hz')
        self.status_label.setStyleSheet('padding: 4px 8px; color: #cbd5e1; background: #171a28; border: 1px solid #32374e; border-radius: 6px;')
        root.addWidget(self.status_label)
        self._set_hp_bar_color('#ef4444')

    def _setup_timer(self):
        self.timer = QtCore.QTimer(self); self.timer.setInterval(UI_INTERVAL_MS)
        self.timer.timeout.connect(self._on_ui_tick); self.timer.start()

    def _on_ui_tick(self):
        now = time.monotonic()
        dt = max(1e-3, now - self.last_ui_tick); inst_hz = 1.0 / dt
        self.ui_hz = self.ui_hz * 0.85 + inst_hz * 0.15 if self.ui_hz > 0 else inst_hz
        self.last_ui_tick = now

        with self.shared.lock:
            gt = self.shared.gimbal_t.get_ordered(); gp = self.shared.gimbal_pitch.get_ordered(); gy = self.shared.gimbal_yaw.get_ordered()
            ct = self.shared.cmd_t.get_ordered(); cvx = self.shared.cmd_vx.get_ordered(); cvy = self.shared.cmd_vy.get_ordered(); cvw = self.shared.cmd_vw.get_ordered()
            ot = self.shared.odom_t.get_ordered(); ovx = self.shared.odom_vx.get_ordered(); ovy = self.shared.odom_vy.get_ordered(); ovw = self.shared.odom_vw.get_ordered()
            lvx=self.shared.latest_vx; lvy=self.shared.latest_vy; lvw=self.shared.latest_vw; cc=self.shared.cmd_count
            lovx=self.shared.latest_odom_vx; lovy=self.shared.latest_odom_vy; lovw=self.shared.latest_odom_vw; los=self.shared.latest_odom_speed
            lfvx=self.shared.latest_final_vx; lfvy=self.shared.latest_final_vy; lfvw=self.shared.latest_final_vw
            gpr=self.shared.game_progress; srt=self.shared.stage_remain_time; chp=self.shared.current_hp; pa=self.shared.projectile_allowance
            thp=dict(self.shared.team_hp); lrx=dict(self.shared.last_rx)

        cutoff = now - WINDOW_SECONDS
        def w(t, v):
            if len(t) == 0: return np.array([]), np.array([])
            m = t >= cutoff; return t[m] - now, v[m]

        self.gimbal_plot.update_data([w(gt, gp), w(gt, gy)])
        self.vx_plot.update_data([w(ct, cvx), w(ot, ovx)])
        self.vy_plot.update_data([w(ct, cvy), w(ot, ovy)])
        self.vw_plot.update_data([w(ct, cvw), w(ot, ovw)])

        sn = GAME_PROGRESS_LABELS[gpr] if 0 <= gpr < len(GAME_PROGRESS_LABELS) else f'\u672a\u77e5({gpr})'
        self.game_stage_label.setText(f'\u9636\u6bb5: {sn}')
        rem = max(0, int(srt)); self.game_remain_label.setText(f'\u5269\u4f59: {rem}s')
        self.game_progress_bar.setValue(min(self.game_progress_bar.maximum(), rem))

        hp = max(0, min(ROBOT_HP_MAX, int(chp))); self.hp_value_label.setText(str(hp)); self.hp_bar.setValue(hp)
        pct = hp / float(ROBOT_HP_MAX)
        self._set_hp_bar_color('#22c55e' if pct > 0.6 else ('#f59e0b' if pct > 0.3 else '#ef4444'))
        self.ammo_label.setText(f'\u5f39\u91cf: {max(0, int(pa))} \u53d1')

        for k, (bar, vl) in self.team_bar_widgets.items():
            v = max(0, min(bar.maximum(), int(thp.get(k, 0)))); bar.setValue(v); vl.setText(str(v))

        self.tx_value_labels['nav_vx'].setText(f'{lvx:+.3f}'); self.tx_value_labels['nav_vy'].setText(f'{lvy:+.3f}')
        self.tx_value_labels['nav_vw'].setText(f'{lvw:+.3f}'); self.tx_count_label.setText(f'\u8ba1\u6570: {cc}')
        rc = int(np.sum(ct >= (now - 2.0))) if len(ct) > 0 else 0
        self.tx_freq_label.setText(f'\u9891\u7387: {rc / 2.0:.1f} Hz')

        for n, lbl in self.tx_value_labels.items():
            v = lvx if n == 'nav_vx' else (lvy if n == 'nav_vy' else lvw)
            self._set_label_color(f'tx_{n}', lbl, '#4f8cff' if abs(v) > 0.01 else '#f8fafc')

        self.final_value_labels['cmd_vx'].setText(f'{lfvx:+.3f}'); self.final_value_labels['cmd_vy'].setText(f'{lfvy:+.3f}')
        self.final_value_labels['cmd_vw'].setText(f'{lfvw:+.3f}')
        for n, lbl in self.final_value_labels.items():
            v = lfvx if n == 'cmd_vx' else (lfvy if n == 'cmd_vy' else lfvw)
            self._set_label_color(f'f_{n}', lbl, '#f59e0b' if abs(v) > 0.01 else '#f8fafc')

        self.actual_value_labels['act_vx'].setText(f'{lovx:+.3f}'); self.actual_value_labels['act_vy'].setText(f'{lovy:+.3f}')
        self.actual_value_labels['act_vw'].setText(f'{lovw:+.3f}'); self.actual_value_labels['speed'].setText(f'{los:.3f}')
        for n, lbl in self.actual_value_labels.items():
            v = los if n == 'speed' else (lovx if n == 'act_vx' else (lovy if n == 'act_vy' else lovw))
            self._set_label_color(f'a_{n}', lbl, '#22c55e' if abs(v) > 0.01 else '#f8fafc')

        evx, evy, evw = lvx - lovx, lvy - lovy, lvw - lovw
        self.track_value_labels['\u0394vx'].setText(f'{evx:+.3f}'); self.track_value_labels['\u0394vy'].setText(f'{evy:+.3f}')
        self.track_value_labels['\u0394vw'].setText(f'{evw:+.3f}')
        for n, lbl in self.track_value_labels.items():
            v = evx if n == '\u0394vx' else (evy if n == '\u0394vy' else evw)
            a = abs(v); c = '#22c55e' if a < 0.1 else ('#fbbf24' if a < 0.5 else '#ef4444')
            self._set_label_color(f't_{n}', lbl, c)

        act = 1.0
        mk = lambda k: '\u2713' if now - lrx.get(k, 0.0) < act else '\u2717'
        self.status_label.setText(
            f'Topics: gimbal {mk("gimbal")} nav_cmd {mk("cmd_vel")} final_cmd {mk("final_cmd")} '
            f'odom {mk("odom")} game {mk("game")} robot {mk("robot")} hp {mk("hp")}   {self.ui_hz:>4.1f}Hz')


def apply_dark_palette(app):
    p = QtGui.QPalette()
    p.setColor(QtGui.QPalette.Window, QtGui.QColor(24, 26, 34))
    p.setColor(QtGui.QPalette.WindowText, QtGui.QColor(235, 238, 245))
    p.setColor(QtGui.QPalette.Base, QtGui.QColor(30, 32, 46))
    p.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(39, 42, 58))
    p.setColor(QtGui.QPalette.ToolTipBase, QtGui.QColor(240, 240, 240))
    p.setColor(QtGui.QPalette.ToolTipText, QtGui.QColor(30, 30, 30))
    p.setColor(QtGui.QPalette.Text, QtGui.QColor(235, 238, 245))
    p.setColor(QtGui.QPalette.Button, QtGui.QColor(36, 39, 52))
    p.setColor(QtGui.QPalette.ButtonText, QtGui.QColor(233, 237, 245))
    p.setColor(QtGui.QPalette.BrightText, QtGui.QColor(255, 100, 100))
    p.setColor(QtGui.QPalette.Highlight, QtGui.QColor(79, 140, 255))
    p.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor(255, 255, 255))
    app.setPalette(p)
    app.setStyleSheet('QWidget { font-size: 13px; } QLabel { color: #e5e7eb; } QMainWindow { background-color: #181a22; }')


def main():
    app = QtWidgets.QApplication(sys.argv); apply_dark_palette(app)
    rclpy.init(); shared = SharedData()
    node = SerialVisualizerNode(shared); executor = SingleThreadedExecutor()
    executor.add_node(node); spin_thread = RosSpinThread(executor); spin_thread.start()
    window = MainWindow(shared); window.show()
    def shutdown():
        if window.timer.isActive(): window.timer.stop()
        executor.shutdown(timeout_sec=0.5); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        spin_thread.join(timeout=1.0)
    app.aboutToQuit.connect(shutdown); sys.exit(app.exec_())


if __name__ == '__main__':
    main()
