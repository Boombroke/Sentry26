#!/bin/bash
# ==============================================================================
# Sentry Nav - 一键环境配置脚本
# 适用系统: Ubuntu 22.04
# ROS 版本: ROS2 Humble
# 用法: bash scripts/setup_env.sh
# ==============================================================================

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC} $1"; }
ok()    { echo -e "${GREEN}[OK]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ---------- 0. 系统检查 ----------
info "检查系统环境..."

if [[ "$(lsb_release -cs 2>/dev/null)" != "jammy" ]]; then
    warn "当前系统非 Ubuntu 22.04 (Jammy)，可能存在兼容性问题"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
info "项目根目录: $PROJECT_DIR"

# ---------- 1. 安装 ROS2 Humble ----------
install_ros2_humble() {
    if command -v ros2 &>/dev/null && ros2 doctor --report 2>/dev/null | grep -q "humble"; then
        ok "ROS2 Humble 已安装"
        return 0
    fi

    info "开始安装 ROS2 Humble..."

    sudo apt update && sudo apt install -y software-properties-common curl

    # 添加 ROS2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # 添加 ROS2 APT 源
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-humble-desktop ros-dev-tools

    ok "ROS2 Humble 安装完成"
}

# ---------- 2. 安装 Gazebo Ignition Fortress ----------
install_gazebo() {
    if command -v ign &>/dev/null || command -v gz &>/dev/null; then
        ok "Gazebo Ignition 已安装"
        return 0
    fi

    info "开始安装 Gazebo Ignition Fortress..."

    sudo apt install -y ros-humble-ros-gz

    ok "Gazebo Ignition Fortress 安装完成"
}

# ---------- 3. 安装系统依赖 ----------
install_system_deps() {
    info "安装系统级依赖..."

    sudo apt update
    sudo apt install -y \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        libeigen3-dev \
        libomp-dev \
        libpcl-dev \
        libunwind-dev \
        libgoogle-glog-dev \
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

    ok "系统依赖安装完成"

    info "安装 Python 工具依赖..."
    pip3 install --quiet pyserial jinja2 2>/dev/null || \
        sudo apt install -y python3-serial python3-jinja2

    ok "Python 工具依赖安装完成"
}

# ---------- 4. 安装 small_gicp ----------
install_small_gicp() {
    if ldconfig -p 2>/dev/null | grep -q "small_gicp"; then
        ok "small_gicp 已安装"
        return 0
    fi

    # 也检查 cmake 配置文件
    if [ -f /usr/local/lib/cmake/small_gicp/small_gicpConfig.cmake ] || \
       [ -f /usr/lib/cmake/small_gicp/small_gicpConfig.cmake ]; then
        ok "small_gicp 已安装 (cmake 配置已找到)"
        return 0
    fi

    info "编译安装 small_gicp..."

    TEMP_DIR=$(mktemp -d)
    cd "$TEMP_DIR"

    git clone --depth 1 --branch v1.0.0 https://github.com/koide3/small_gicp.git
    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j"$(nproc)"
    sudo make install
    sudo ldconfig

    cd "$PROJECT_DIR"
    rm -rf "$TEMP_DIR"

    ok "small_gicp v1.0.0 安装完成"
}

# ---------- 5. 初始化 rosdep ----------
init_rosdep() {
    info "初始化 rosdep..."

    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init 2>/dev/null || true
    fi
    rosdep update --rosdistro=humble

    ok "rosdep 初始化完成"
}

# ---------- 6. 创建工作空间并编译 ----------
build_workspace() {
    info "配置 ROS2 工作空间..."

    # source ROS2 环境
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash

    WS_DIR="${WS_DIR:-$HOME/sentry_ws}"

    if [ -d "$WS_DIR/src" ]; then
        warn "工作空间 $WS_DIR 已存在，跳过创建"
    else
        mkdir -p "$WS_DIR/src"
        info "已创建工作空间: $WS_DIR"
    fi

    # 链接/复制源码（PROJECT_DIR 即为 src/ 目录，直接链接其中的包）
    if [ ! -d "$WS_DIR/src/sentry_nav_bringup" ]; then
        info "将源码链接到工作空间..."
        ln -sf "$PROJECT_DIR/"* "$WS_DIR/src/" 2>/dev/null || \
            cp -rs "$PROJECT_DIR/"* "$WS_DIR/src/"
    fi

    # 安装 ROS 依赖
    info "安装 ROS 包依赖 (rosdep)..."
    cd "$WS_DIR"
    rosdep install -r --from-paths src --ignore-src --rosdistro humble -y 2>/dev/null || \
        warn "部分 rosdep 依赖未能自动安装，可能需要手动处理"

    # 编译
    info "开始编译 (colcon build)..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

    ok "编译完成!"
    echo ""
    info "请执行以下命令加载环境:"
    echo -e "  ${GREEN}source $WS_DIR/install/setup.bash${NC}"
    echo ""
    info "仿真模式启动:"
    echo -e "  ${GREEN}ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py${NC}"
    echo ""
}

# ---------- 7. 配置环境变量 (可选) ----------
setup_bashrc() {
    local SETUP_LINE="source $WS_DIR/install/setup.bash"

    if grep -qF "$SETUP_LINE" ~/.bashrc 2>/dev/null; then
        ok "bashrc 已配置"
        return 0
    fi

    read -rp "是否将工作空间写入 ~/.bashrc? [y/N] " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        {
            echo ""
            echo "# Sentry Nav ROS2 工作空间"
            echo "source /opt/ros/humble/setup.bash"
            echo "$SETUP_LINE"
        } >> ~/.bashrc
        ok "已写入 ~/.bashrc"
    fi
}

# ==================== 主流程 ====================
echo ""
echo "============================================"
echo "  Sentry Nav - 一键环境配置"
echo "  Ubuntu 22.04 + ROS2 Humble"
echo "============================================"
echo ""

install_ros2_humble
install_gazebo
install_system_deps
install_small_gicp
init_rosdep
build_workspace
setup_bashrc

echo ""
echo "============================================"
ok "全部配置完成!"
echo "============================================"
