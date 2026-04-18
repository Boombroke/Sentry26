#!/bin/bash
# record_nav_bag.sh - 录制 Nav Goal 调试 rosbag 到项目 logs/
# 用法:
#   bash src/scripts/debug/record_nav_bag.sh                          # 默认 30s, Goal (3,2)
#   bash src/scripts/debug/record_nav_bag.sh baseline 60              # 自定义标签和时长
#   bash src/scripts/debug/record_nav_bag.sh tuned 45 2.0 0.0         # 加目标 xy

set -e

LABEL="${1:-run}"
DURATION="${2:-30}"
GOAL_X="${3:-3.0}"
GOAL_Y="${4:-2.0}"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOGS_DIR="$PROJECT_ROOT/logs"
mkdir -p "$LOGS_DIR"

TS=$(date +%Y%m%d-%H%M%S)
BAG_DIR="$LOGS_DIR/${TS}-${LABEL}"

echo "录制目标: $BAG_DIR"
echo "时长: ${DURATION}s"
echo "Nav Goal: ($GOAL_X, $GOAL_Y)"
echo "预计体积: ${DURATION}s × (livox/lidar ~3MB/s + 其余 ~0.5MB/s) ≈ $((DURATION * 35 / 10))MB"
echo ""

NS="/red_standard_robot1"
# 速度指令链路 (cmd_vel_controller = RPP 原始, cmd_vel_smoothed = smoother 输出, cmd_vel = 最终)
TOPICS_CMD="$NS/cmd_vel $NS/cmd_vel_controller $NS/cmd_vel_smoothed"

# 定位 / 里程计 / TF
TOPICS_POSE="$NS/odometry $NS/aft_mapped_to_init $NS/joint_states $NS/tf $NS/tf_static"

# 规划器 / 控制器 (local_plan 来自 RPP, transformed_global_plan 来自 RPP, received_global_plan 来自 BT)
TOPICS_NAV="$NS/plan $NS/local_plan $NS/transformed_global_plan $NS/received_global_plan $NS/lookahead_point"

# Costmap (带 _raw 未 inflate, 不带是 inflate 后; 订 _raw 数据量小, 重放自己 inflate)
TOPICS_COSTMAP="$NS/local_costmap/costmap $NS/global_costmap/costmap $NS/local_costmap/published_footprint"

# 感知输入 (terrain → costmap observation, lidar → SLAM/LIO, imu → LIO)
TOPICS_SENSE="$NS/terrain_map $NS/terrain_map_ext $NS/obstacle_scan $NS/livox/imu $NS/livox/lidar"

# BT / behavior 状态 (recovery 触发看这几个)
TOPICS_BT="$NS/behavior_tree_log $NS/navigate_to_pose/_action/status"

TOPICS="$TOPICS_CMD $TOPICS_POSE $TOPICS_NAV $TOPICS_COSTMAP $TOPICS_SENSE $TOPICS_BT"

# 在 bash 子 shell 里跑 ros2 bag, 静默 INFO, 只保留 WARN/ERROR
( ros2 bag record --output "$BAG_DIR" -d "$DURATION" --topics $TOPICS 2>&1 | grep -v "INFO" ) &
BAG_PID=$!

# 稍等录制起来
sleep 3

# 发 Goal（只保留关键输出）
echo "发 Nav Goal..."
ros2 action send_goal "$NS/navigate_to_pose" nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {w: 1.0}}}}" 2>&1 \
    | grep -E "Goal|SUCCEEDED|ABORTED|REJECTED|error_code" || echo "Goal action 发送失败，检查 Nav2 是否已 active"

# 等 bag 结束
wait $BAG_PID 2>/dev/null || true

# 自动 reindex (Ctrl+C 中断的 bag 会缺 metadata)
if [ ! -f "$BAG_DIR/metadata.yaml" ]; then
    echo "自动 reindex..."
    ros2 bag reindex "$BAG_DIR" 2>&1 | tail -2
fi

echo ""
echo "=== 完成 ==="
echo "bag 位置: $BAG_DIR"
du -sh "$BAG_DIR" 2>/dev/null
echo ""
echo "分析:"
echo "  python3 $PROJECT_ROOT/src/scripts/debug/analyze_bag.py $BAG_DIR"
