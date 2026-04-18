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
echo ""

NS="/red_standard_robot1"
TOPICS="$NS/cmd_vel $NS/cmd_vel_controller $NS/odometry $NS/joint_states $NS/plan $NS/local_plan $NS/tf $NS/tf_static $NS/local_costmap/costmap $NS/livox/imu"

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
