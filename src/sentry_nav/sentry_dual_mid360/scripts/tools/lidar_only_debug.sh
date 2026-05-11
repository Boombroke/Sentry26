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
#   --with-pointlio   额外起 Point-LIO，发 /cloud_registered (PointCloud2)
#                     rviz 能直接看的 PC2 点云。livox CustomMsg 本身 rviz
#                     无法 render，加这个才能看到实际融合点云。隐含
#                     --with-merger（Point-LIO 消费 /livox/lidar）
#   --no-rviz         不起 rviz2（默认会起；ssh/无屏环境/已开 rviz 时关掉）
#   --no-driver       不起 livox driver（你在别处已经起好了）
#   --no-rsp          不起 robot_state_publisher（同上）
#   --help

set -euo pipefail

WITH_MERGER="no"
WITH_POINTLIO="no"
WITH_RVIZ="yes"   # 默认起 rviz；--no-rviz 可关
NO_DRIVER="no"
NO_RSP="no"

while [ $# -gt 0 ]; do
    case "$1" in
        --with-merger)   WITH_MERGER="yes"; shift ;;
        --with-pointlio) WITH_POINTLIO="yes"; WITH_MERGER="yes"; shift ;;
        --with-rviz)     WITH_RVIZ="yes"; shift ;;   # 兼容旧调用，等价于默认
        --no-rviz)       WITH_RVIZ="no";  shift ;;
        --no-driver)     NO_DRIVER="yes"; shift ;;
        --no-rsp)        NO_RSP="yes"; shift ;;
        --help|-h)
            sed -n '2,29p' "$0"
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

if [ "$WITH_POINTLIO" = "yes" ]; then
    echo "[INFO] starting Point-LIO (consumes /livox/lidar + /livox/imu,"
    echo "       publishes /cloud_registered as PointCloud2 for rviz)..."
    ros2 launch point_lio point_lio.launch.py rviz:=False &
    PIDS+=($!)
    # Point-LIO 默认把点云发到 camera_init 系、内部发 camera_init → aft_mapped
    # 的 TF。生产栈靠 odom_bridge 再把它桥到 lidar_odom/map；我们不起
    # odom_bridge，直接补一条 identity static TF 让 rviz 能把 /cloud_registered
    # 变换到 map 里显示。
    echo "[INFO] fake map -> camera_init (identity, lets rviz render /cloud_registered)..."
    ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
        --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id camera_init &
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

注意: /livox/lidar_front、/livox/lidar_back、/livox/lidar 都是
livox_ros_driver2/CustomMsg，rviz 无法 render (Add 列表里灰色无法添加)。

要在 rviz 里看到实际点云，加 --with-pointlio，Point-LIO 会发
/cloud_registered (PointCloud2)：

$(if [ "$WITH_POINTLIO" = "yes" ]; then
cat <<'HINT'
In rviz:
  Fixed Frame: map   (手打，rviz 下拉框里可能没有，直接编辑该字段)
  Add -> PointCloud2  -> Topic: /cloud_registered
      (Point-LIO 的世界系融合点云，frame_id=camera_init)
      Size (Pixels): 3
      Color Transformer: Intensity 或 AxisColor
      Decay Time: 0
  Add -> Odometry     -> Topic: /aft_mapped_to_init (Point-LIO 里程计)
  Add -> TF           (看 map -> camera_init -> aft_mapped 是否连通)

判断标定好坏:
  看 /cloud_registered 里墙是薄一层、立柱是一根、地面单平面 = 标定OK
  双层墙 / 错位柱 = xmacro 里 front/back_lidar_pose 还有偏差

注意：Point-LIO 原生发 camera_init 系，生产栈靠 odom_bridge 桥到
lidar_odom/map；这里没起 odom_bridge，我们补了一条 identity static TF
map -> camera_init 让 rviz 能在 map 系里显示点云。
HINT
else
cat <<'HINT'
当前没起 Point-LIO，只能用 ros2 topic echo / ros2 topic hz 看 raw 数据。
想在 rviz 里看到双雷达叠加效果，Ctrl-C 后重跑并加 --with-pointlio。
HINT
fi)

Ctrl-C 停止所有进程。
================================================================
EOF

# Block forever; trap will clean up on Ctrl-C.
wait
