#!/bin/bash
set -e
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
info() { echo -e "${CYAN}[INFO]${NC} $1"; }
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
err()  { echo -e "${RED}[ERROR]${NC} $1"; }

if [ -f /opt/ros/humble/setup.bash ]; then
    ok "ROS2 Humble 已安装"
    info "请执行: source /opt/ros/humble/setup.bash"
    exit 0
fi

info "ROS2 Humble 未安装，开始安装..."
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
ok "ROS2 Humble 安装完成，请执行: source /opt/ros/humble/setup.bash"
