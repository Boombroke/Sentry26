#!/bin/bash
# lidar_only_debug.sh
#
# 一键起"只看激光"的最小调试栈，不依赖下位机 serial、nav2、slam。
# 适用场景：
#   - T11 标定完想在 rviz 里肉眼验证前后雷达点云叠加质量
#   - 下位机 / 串口没接，只想看 /livox/lidar_{front,back,合并} 是否正常
#   - 排查 merger 输出是否"墙成薄一层、立柱成一根"
#
# 起的东西：
#   1. sentry_dual_mid360 dual_mid360_driver_launch.py  （两路 CustomMsg 驱动）
#   2. sentry_nav_bringup  robot_state_publisher_launch.py  （xmacro 静态 TF）
#   3. static_transform_publisher  map  -> odom           （identity，替代 small_gicp）
#   4. static_transform_publisher  odom -> base_footprint （identity，替代 Point-LIO）
#   5. (可选) sentry_dual_mid360 pointcloud_merger_launch.py  （发 /livox/lidar 融合）
#   6. (可选) rviz2
#
# Ctrl-C 会把所有子进程一起杀掉，不留僵尸。
#
# Flags (defaults are the "一键调试" common case):
#   --with-merger     额外起 pointcloud_merger，看合并后的 /livox/lidar
#   --no-rviz         不起 rviz2（默认会起；ssh/无屏环境/已开 rviz 时关掉）
#   --no-driver       不起 livox driver（你在别处已经起好了）
#   --no-rsp          不起 robot_state_publisher（同上）
#   --help

set -euo pipefail

WITH_MERGER="no"
WITH_RVIZ="yes"   # 默认起 rviz；--no-rviz 可关
NO_DRIVER="no"
NO_RSP="no"

while [ $# -gt 0 ]; do
    case "$1" in
        --with-merger) WITH_MERGER="yes"; shift ;;
        --with-rviz)   WITH_RVIZ="yes"; shift ;;   # 兼容旧调用，等价于默认
        --no-rviz)     WITH_RVIZ="no";  shift ;;
        --no-driver)   NO_DRIVER="yes"; shift ;;
        --no-rsp)      NO_RSP="yes"; shift ;;
        --help|-h)
            sed -n '2,25p' "$0"
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

echo "[INFO] fake map->odom (identity, replaces small_gicp_relocalization)..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom &
PIDS+=($!)

echo "[INFO] fake odom->base_footprint (identity, replaces Point-LIO)..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
    --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id base_footprint &
PIDS+=($!)

if [ "$WITH_MERGER" = "yes" ]; then
    echo "[INFO] starting pointcloud_merger (/livox/lidar)..."
    ros2 launch sentry_dual_mid360 pointcloud_merger_launch.py &
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

In rviz:
  Fixed Frame: base_footprint  (或 front_mid360 直接看原始点云)
  Add -> PointCloud2  -> Topic: /livox/lidar_front
  Add -> PointCloud2  -> Topic: /livox/lidar_back
$(if [ "$WITH_MERGER" = "yes" ]; then echo "  Add -> PointCloud2  -> Topic: /livox/lidar        (merger 输出，换个颜色看)"; fi)
  Add -> TF           (确认 map -> odom -> base_footprint -> ... -> front_mid360 全连通)

Ctrl-C 停止所有进程。
================================================================
EOF

# Block forever; trap will clean up on Ctrl-C.
wait
