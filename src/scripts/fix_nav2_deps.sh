#!/bin/bash
set -e
GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
info() { echo -e "${CYAN}[INFO]${NC} $1"; }
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }

source /opt/ros/jazzy/setup.bash 2>/dev/null || { echo "请先安装 ROS2: bash src/scripts/fix_ros_env.sh"; exit 1; }

info "安装 Nav2 及导航相关依赖..."
sudo apt update
sudo apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-serial-driver \
    ros-jazzy-joy
ok "Nav2 依赖安装完成"
