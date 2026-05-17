#!/bin/bash
# Launch serial_visualizer with a clean ROS overlay (avoids stale paths in install/setup.bash).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
else
  echo "ERROR: ROS2 Jazzy not found at /opt/ros/jazzy" >&2
  exit 1
fi

if [[ -f "$WS_ROOT/install/local_setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_ROOT/install/local_setup.bash"
else
  echo "WARN: $WS_ROOT/install/local_setup.bash missing — build workspace first:" >&2
  echo "  cd $WS_ROOT && colcon build --symlink-install" >&2
fi

exec python3 "$SCRIPT_DIR/serial_visualizer.py" "$@"
