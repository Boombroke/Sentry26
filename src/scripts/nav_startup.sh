#!/bin/bash
# cp src/scripts/nav_startup.sh ~/Desktop/

SENTRY_WS="/home/sentry01/Documents/Sentry26"
LOG="$SENTRY_WS/log/nav_startup.log"

mkdir -p "$SENTRY_WS/log"
exec >>"$LOG" 2>&1
echo "===== $(date '+%F %T') nav_startup ====="

sleep 15

cd "$SENTRY_WS" || exit 1
unset LD_LIBRARY_PATH

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source install/setup.bash

export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True
