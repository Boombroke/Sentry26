#!/usr/bin/env python3
"""分析 Nav Goal 调试 rosbag，输出控制跟随性、打滑、摇摆等指标。

用法:
    python3 analyze_bag.py [bag_dir]
    默认分析项目根的 nav_debug/ 或 logs/ 下最新一个 bag。
"""
import sys
import os
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
import numpy as np


def find_latest_bag():
    """优先 logs/最新，其次 nav_debug/"""
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))
    logs = os.path.join(root, 'logs')
    if os.path.isdir(logs):
        cands = sorted(
            [os.path.join(logs, d) for d in os.listdir(logs)
             if os.path.isdir(os.path.join(logs, d)) and
             any(f.endswith('.mcap') or f.endswith('.db3') for f in os.listdir(os.path.join(logs, d)))],
            reverse=True)
        if cands:
            return cands[0]
    nav_debug = os.path.join(root, 'nav_debug')
    if os.path.isdir(nav_debug):
        return nav_debug
    return None


BAG_PATH = sys.argv[1] if len(sys.argv) > 1 else find_latest_bag()
if BAG_PATH is None or not os.path.isdir(BAG_PATH):
    print(f"未找到 bag 目录: {BAG_PATH}")
    print("用法: python3 analyze_bag.py <bag_dir>")
    sys.exit(1)

# 若没 metadata.yaml（录制被 Ctrl+C 中断），自动 reindex
if not os.path.exists(os.path.join(BAG_PATH, 'metadata.yaml')):
    print(f"缺 metadata.yaml，自动 reindex...")
    import subprocess
    r = subprocess.run(['ros2', 'bag', 'reindex', BAG_PATH],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"reindex 失败: {r.stderr}")
        sys.exit(1)
    print("reindex 完成")

print(f"分析 bag: {BAG_PATH}")

reader = rosbag2_py.SequentialReader()
# 自动检测 mcap / sqlite3
storage_id = 'mcap' if any(f.endswith('.mcap') for f in os.listdir(BAG_PATH)) else 'sqlite3'
storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id=storage_id)
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

cmd_vel_raw = []
cmd_vel_final = []
odom = []
imu = []

while reader.has_next():
    topic, data, t = reader.read_next()
    t_sec = t / 1e9
    if 'cmd_vel_controller' in topic:
        m = deserialize_message(data, TwistStamped)
        cmd_vel_raw.append((t_sec, m.twist.linear.x, m.twist.linear.y, m.twist.angular.z))
    elif topic.endswith('/cmd_vel'):
        m = deserialize_message(data, TwistStamped)
        cmd_vel_final.append((t_sec, m.twist.linear.x, m.twist.linear.y, m.twist.angular.z))
    elif 'odometry' in topic:
        m = deserialize_message(data, Odometry)
        odom.append((t_sec,
                     m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z,
                     m.pose.pose.position.x, m.pose.pose.position.y))
    elif 'livox/imu' in topic:
        m = deserialize_message(data, Imu)
        imu.append((t_sec,
                    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))

cmd_raw = np.array(cmd_vel_raw) if cmd_vel_raw else np.empty((0, 4))
cmd_fin = np.array(cmd_vel_final) if cmd_vel_final else np.empty((0, 4))
od = np.array(odom) if odom else np.empty((0, 6))
im = np.array(imu) if imu else np.empty((0, 7))

if len(od) == 0 or len(cmd_fin) == 0:
    print("bag 里没有 odometry 或 cmd_vel，无法分析")
    sys.exit(1)

print(f"\n=== 数据规模 ===")
print(f"时长: {od[-1,0] - od[0,0]:.1f}s  "
      f"cmd_vel_final: {len(cmd_fin)}  odometry: {len(od)}  imu: {len(im)}")

def interp(src, t_target, col):
    return np.interp(t_target, src[:, 0], src[:, col])

t_common = od[:, 0]
cmd_vx_at_odom = interp(cmd_fin, t_common, 1)
cmd_wz_at_odom = interp(cmd_fin, t_common, 3)
od_vx = od[:, 1]
od_vy = od[:, 2]
od_wz = od[:, 3]

vx_err = od_vx - cmd_vx_at_odom
wz_err = od_wz - cmd_wz_at_odom
print(f"\n=== 1. 速度跟随性 ===")
print(f"  cmd vx: [{cmd_fin[:,1].min():.2f}, {cmd_fin[:,1].max():.2f}] 均值 {cmd_fin[:,1].mean():.2f}")
print(f"  odom vx: [{od_vx.min():.2f}, {od_vx.max():.2f}] 均值 {od_vx.mean():.2f}")
print(f"  vx 跟随误差 RMS: {np.sqrt((vx_err**2).mean()):.3f}")
print(f"  cmd wz: [{cmd_fin[:,3].min():.2f}, {cmd_fin[:,3].max():.2f}]")
print(f"  odom wz: [{od_wz.min():.2f}, {od_wz.max():.2f}]")
print(f"  wz 跟随误差 RMS: {np.sqrt((wz_err**2).mean()):.3f}")

print(f"\n=== 2. 横向打滑（差速应为 0）===")
print(f"  odom vy: [{od_vy.min():.3f}, {od_vy.max():.3f}]  RMS {np.sqrt((od_vy**2).mean()):.3f} m/s")

if len(im) > 0:
    print(f"\n=== 3. 前后摇摆 ===")
    print(f"  IMU wy (pitch rate) RMS: {np.sqrt((im[:,2]**2).mean()):.3f} rad/s")
    print(f"  IMU az std: {im[:,6].std():.3f}  (<0.5 稳，>2 震荡，>5 物理发散)")

rotating = (np.abs(cmd_fin[:,3]) > 0.3) & (np.abs(cmd_fin[:,1]) < 0.1)
straight = (np.abs(cmd_fin[:,1]) > 0.3) & (np.abs(cmd_fin[:,3]) < 0.1)
combined = (np.abs(cmd_fin[:,3]) > 0.1) & (np.abs(cmd_fin[:,1]) > 0.1)
idle = (np.abs(cmd_fin[:,1]) < 0.05) & (np.abs(cmd_fin[:,3]) < 0.05)
print(f"\n=== 4. 控制策略占比 ===")
print(f"  只转不走: {rotating.mean()*100:.1f}%   只走不转: {straight.mean()*100:.1f}%")
print(f"  边转边走: {combined.mean()*100:.1f}%   完全停止: {idle.mean()*100:.1f}%")

dx = np.diff(od[:,4]); dy = np.diff(od[:,5])
path_len = np.sum(np.sqrt(dx**2+dy**2))
dist = np.sqrt((od[-1,4]-od[0,4])**2 + (od[-1,5]-od[0,5])**2)
print(f"\n=== 5. 路径 ===")
print(f"  起点 ({od[0,4]:.2f}, {od[0,5]:.2f}) → 终点 ({od[-1,4]:.2f}, {od[-1,5]:.2f})")
print(f"  直线距离 {dist:.2f}m / 实际路径 {path_len:.2f}m / 效率 {dist/path_len*100:.0f}%")

# 健康度打分
print(f"\n=== 健康度评分（越低越好，>5 有问题，>10 很差）===")
score_slip = min(10, np.sqrt((od_vy**2).mean()) * 10)
score_phys = min(10, im[:,6].std() if len(im) > 0 else 0)
score_rotate = min(10, rotating.mean() * 20)
score_path = min(10, max(0, (1 - dist/path_len) * 20))
print(f"  打滑: {score_slip:.1f}   物理稳定: {score_phys:.1f}   "
      f"原地旋转: {score_rotate:.1f}   绕路: {score_path:.1f}")
print(f"  总分: {score_slip + score_phys + score_rotate + score_path:.1f}/40")
