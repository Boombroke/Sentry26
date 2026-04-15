#!/bin/bash
set -e
GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
info() { echo -e "${CYAN}[INFO]${NC} $1"; }
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }

source /opt/ros/humble/setup.bash 2>/dev/null || { echo "请先安装 ROS2: bash src/scripts/fix_ros_env.sh"; exit 1; }

info "安装 Nav2 及导航相关依赖..."
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf2-eigen \
    ros-humble-serial-driver \
    ros-humble-joy
ok "Nav2 依赖安装完成"
