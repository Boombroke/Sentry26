#!/bin/bash
# lidar_only_debug.sh
#
# 一键起"只看激光"的最小调试栈，不依赖下位机 serial、nav2、slam、Point-LIO。
# 适用场景：
#   - T11 标定完想在 rviz 里肉眼验证前后雷达点云叠加质量
#   - 下位机 / 串口没接，只想看 /livox/lidar_{front,back,合并} 是否正常
#   - 排查 merger 输出是否"墙成薄一层、立柱成一根"
#
# 起的东西：
#   1. sentry_dual_mid360 dual_mid360_driver_launch.py  （两路 CustomMsg 驱动）
#   2. sentry_nav_bringup  robot_state_publisher_launch.py  （xmacro 静态 TF）
#   3. static_transform_publisher  map  -> odom           （identity）
#   4. static_transform_publisher  odom -> base_footprint （identity）
#   5. sentry_dual_mid360 pointcloud_merger_launch.py (publish_pc2_preview:=true)
#      同时发 /livox/lidar (CustomMsg) 和 /livox/lidar_pc2 (PointCloud2)
#   6. rviz2
#
# 整车验证走 rm_sentry_launch.py（完整 Point-LIO + SLAM + Nav2）；本脚本
# 专为"桌面摆两颗雷达"这种无整车调试场景设计，不起 Point-LIO——裸摆
# IMU 条件不稳定 ESKF 经常不收敛，对验证双雷达外参帮助有限。
#
# Ctrl-C 会把所有子进程一起杀掉，不留僵尸。
#
# Flags (defaults 已经是常见用例):
#   --no-merger       不起 pointcloud_merger；默认会起并开 PC2 mirror
#   --no-rviz         不起 rviz2（默认会起；ssh/无屏环境/已开 rviz 时关掉）
#   --no-driver       不起 livox driver（你在别处已经起好了）
#   --no-rsp          不起 robot_state_publisher（同上）
#   --help

set -euo pipefail

WITH_MERGER="yes"  # 默认带 merger；--no-merger 关掉
WITH_RVIZ="yes"    # 默认起 rviz；--no-rviz 可关
NO_DRIVER="no"
NO_RSP="no"

while [ $# -gt 0 ]; do
    case "$1" in
        --with-merger)   WITH_MERGER="yes"; shift ;;   # 兼容旧调用
        --no-merger)     WITH_MERGER="no"; shift ;;
        --with-rviz)     WITH_RVIZ="yes"; shift ;;     # 兼容旧调用
        --no-rviz)       WITH_RVIZ="no";  shift ;;
        --no-driver)     NO_DRIVER="yes"; shift ;;
        --no-rsp)        NO_RSP="yes"; shift ;;
        --help|-h)
            sed -n '2,32p' "$0"
            exit 0
            ;;
        *)
            echo "[ERROR] unknown option: $1" >&2
            exit 2
            ;;
    esac
done

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[ERROR] ros2 not on PATH. source /opt/ros/jazzy/setup.bash and install/setup.bash first." >&2
    exit 1
fi

PIDS=()
cleanup() {
    echo ""
    echo "[INFO] shutting down (pids: ${PIDS[*]})..."
    # SIGINT first, give 3s, then SIGTERM holdouts.
    for pid in "${PIDS[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    sleep 3
    for pid in "${PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[INFO] all stopped."
}
trap cleanup INT TERM EXIT

if [ "$NO_DRIVER" = "no" ]; then
    echo "[INFO] starting livox driver..."
    ros2 launch sentry_dual_mid360 dual_mid360_driver_launch.py &
    PIDS+=($!)
fi

if [ "$NO_RSP" = "no" ]; then
    echo "[INFO] starting robot_state_publisher (xmacro static TF)..."
    ros2 launch sentry_nav_bringup robot_state_publisher_launch.py &
    PIDS+=($!)
fi

echo "[INFO] fake map->odom (identity)..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom &
PIDS+=($!)

echo "[INFO] fake odom->base_footprint (identity)..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id base_footprint &
PIDS+=($!)

if [ "$WITH_MERGER" = "yes" ]; then
    # publish_pc2_preview:=true lets the merger mirror /livox/lidar (CustomMsg)
    # as /livox/lidar_pc2 (sensor_msgs/PointCloud2). rviz can't render
    # CustomMsg, so this mirror is what you Add in rviz to eyeball calibration.
    echo "[INFO] starting pointcloud_merger (/livox/lidar + /livox/lidar_pc2 mirror)..."
    ros2 launch sentry_dual_mid360 pointcloud_merger_launch.py \
        publish_pc2_preview:=true &
    PIDS+=($!)
fi

if [ "$WITH_RVIZ" = "yes" ]; then
    echo "[INFO] starting rviz2..."
    rviz2 &
    PIDS+=($!)
fi

cat <<EOF

================================================================
[INFO] lidar-only debug stack up. Processes started: ${#PIDS[@]}

$(if [ "$WITH_MERGER" = "yes" ]; then
cat <<'HINT'
In rviz:
  Fixed Frame: front_mid360      (手打到 Global Options > Fixed Frame)
  Add -> PointCloud2  -> Topic: /livox/lidar_pc2
      merger 输出的 sensor_msgs/PointCloud2 镜像，intensity=reflectivity
      Size (Pixels): 3
      Color Transformer: Intensity (推荐) 或 AxisColor
      Decay Time: 0

判断标定好坏:
  - 墙体是薄的一层、立柱单根、地面单平面 = 外参对
  - 双层墙 / 错位柱 / V 字形折角 = xmacro 里 front/back_lidar_pose 还偏

想要 Point-LIO 的 /cloud_registered 或整车级验证: 请直接走
  ros2 launch sentry_nav_bringup rm_sentry_launch.py
它带完整 Point-LIO + SLAM + Nav2 + odom_bridge，有整车 IMU 稳定条件
ESKF 能收敛。桌面摆雷达模式不要起 Point-LIO，外参问题看 /livox/lidar_pc2
已足够。
HINT
else
cat <<'HINT'
当前没起 merger，只有 driver + TF。
ros2 topic echo / ros2 topic hz 能看 raw 数据，但 rviz 里看不到点云。
删掉 --no-merger 就能在 rviz 里看到 /livox/lidar_pc2。
HINT
fi)

Ctrl-C 停止所有进程。
================================================================
EOF

# Block forever; trap will clean up on Ctrl-C.
wait
