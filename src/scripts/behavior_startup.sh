#!/bin/bash
# 行为树决策自启（需在 serial_startup + nav_startup 之后）
# 复制到桌面: cp src/scripts/behavior_startup.sh ~/Desktop/
# GNOME 命令:
#   gnome-terminal --title="Behavior" -- bash -lc "/home/sentry01/Desktop/behavior_startup.sh; exec bash"

SENTRY_WS="/home/sentry01/Documents/Sentry26"
LOG="$SENTRY_WS/log/behavior_startup.log"
TARGET_TREE="${TARGET_TREE:-red}"

mkdir -p "$SENTRY_WS/log"
exec >>"$LOG" 2>&1
echo "===== $(date '+%F %T') behavior_startup tree=$TARGET_TREE ====="

sleep 20

cd "$SENTRY_WS" || exit 1
unset LD_LIBRARY_PATH

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source install/setup.bash

exec ros2 launch sentry_behavior sentry_behavior_launch.py \
  use_sim_time:=false \
  target_tree:="$TARGET_TREE"
