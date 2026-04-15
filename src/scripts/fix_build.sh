#!/bin/bash
set -e
GREEN='\033[0;32m'; CYAN='\033[0;36m'; YELLOW='\033[1;33m'; NC='\033[0m'
info() { echo -e "${CYAN}[INFO]${NC} $1"; }
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

info "工作空间: $WS_DIR"

source /opt/ros/humble/setup.bash 2>/dev/null || { echo "请先安装 ROS2: bash src/scripts/fix_ros_env.sh"; exit 1; }

info "安装 rosdep 依赖..."
cd "$WS_DIR"
if ! command -v rosdep &>/dev/null; then
    sudo apt install -y python3-rosdep
    sudo rosdep init 2>/dev/null || true
    rosdep update --rosdistro=humble
fi
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y 2>/dev/null || \
    warn "部分依赖未能自动安装"

info "清理旧编译缓存..."
rm -rf build/ install/ log/

info "全量编译..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

ok "编译完成，请执行: source $WS_DIR/install/setup.bash"
