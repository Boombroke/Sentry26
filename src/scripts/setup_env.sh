#!/bin/bash
# ==============================================================================
# Sentry Nav - 一键环境配置脚本
# 适用系统: Ubuntu 24.04
# ROS 版本: ROS2 Jazzy
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

if [[ "$(lsb_release -cs 2>/dev/null)" != "noble" ]]; then
    warn "当前系统非 Ubuntu 24.04 (Noble)，可能存在兼容性问题"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
info "项目根目录: $PROJECT_DIR"

# ---------- 1. 安装 ROS2 Jazzy ----------
install_ros2_jazzy() {
    if command -v ros2 &>/dev/null && ros2 doctor --report 2>/dev/null | grep -q "jazzy"; then
        ok "ROS2 Jazzy 已安装"
        return 0
    fi

    info "开始安装 ROS2 Jazzy..."

    sudo apt update && sudo apt install -y software-properties-common curl

    # 添加 ROS2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # 添加 ROS2 APT 源
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-jazzy-desktop ros-dev-tools

    ok "ROS2 Jazzy 安装完成"
}

# ---------- 2. 安装 Gazebo Harmonic ----------
install_gazebo() {
    if command -v gz &>/dev/null; then
        ok "Gazebo Harmonic 已安装"
        return 0
    fi

    info "开始安装 Gazebo Harmonic..."

    sudo apt install -y ros-jazzy-ros-gz

    ok "Gazebo Harmonic 安装完成"
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

    ok "系统依赖安装完成"

    info "安装 Python 工具依赖..."
    pip3 install --quiet pyserial jinja2 xmacro --break-system-packages 2>/dev/null || \
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

# ---------- 5. Multi_LiCa 标定工具依赖（可选，双雷达外参标定用） ----------
# Multi_LiCa: https://github.com/TUMFTM/Multi_LiCa
# 用途: 双 Mid360 LiDAR-to-LiDAR 外参标定（T11 阶段使用）
# 源码位置: src/third_party/Multi_LiCa/（已克隆，含 TEASER-plusplus 子模块）
#
# 依赖说明:
#   - libomp-dev          : OpenMP 并行支持（TEASER++ 编译必需）
#   - libgtest-dev        : Google Test（TEASER++ 单元测试，可选）
#   - cmake / build-essential : 已在步骤 3 安装
#   - Python: open3d, scipy, ros2_numpy, pandas（见 requirements.txt）
#   - TEASER++: 需从子模块源码编译 Python 绑定（见下方说明）
#
# 注意: Multi_LiCa 不加入默认 colcon 全量编译，避免 TEASER++ 重型依赖
#       影响主导航栈构建。标定时单独在 Multi_LiCa 目录内构建。
#       本函数默认跳过；设置 INSTALL_MULTI_LICA_DEPS=1 后才执行。
#
# TEASER++ 手动编译步骤（仅标定时需要，非日常开发必需）:
#   cd src/third_party/Multi_LiCa/TEASER-plusplus
#   mkdir build && cd build
#   cmake -DTEASERPP_PYTHON_VERSION=3.$(python3 -c "import sys; print(sys.version_info.minor)") ..
#   make teaserpp_python -j$(nproc)
#   cd python && pip install . --break-system-packages
#
# 已知问题（见 TUMFTM/Multi_LiCa#17）:
#   - TEASER++ 子模块 v2.0 在某些环境下 Python 绑定编译失败
#   - 修复: pip install --upgrade pip 后在 TEASER-plusplus/ 根目录执行 pip install .
#   - scipy >= 1.6 的 cKDTree.query() n_jobs 参数已改为 workers，需注意版本兼容
install_multi_lica_deps() {
    if [ "${INSTALL_MULTI_LICA_DEPS:-0}" != "1" ]; then
        info "跳过 Multi_LiCa 依赖安装（如需安装，设置 INSTALL_MULTI_LICA_DEPS=1 后重新运行）"
        return 0
    fi

    info "安装 Multi_LiCa 标定工具依赖..."

    sudo apt install -y \
        libomp-dev \
        libgtest-dev

    pip3 install --quiet \
        open3d \
        scipy \
        ros2_numpy \
        pandas \
        "setuptools==58.2" \
        --break-system-packages 2>/dev/null || \
        warn "部分 Multi_LiCa Python 依赖安装失败，标定时请手动安装"

    ok "Multi_LiCa 依赖安装完成（TEASER++ Python 绑定需手动编译，见注释）"
}

# ---------- 6. 初始化 rosdep ----------
init_rosdep() {
    info "初始化 rosdep..."

    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init 2>/dev/null || true
    fi
    rosdep update --rosdistro=jazzy

    ok "rosdep 初始化完成"
}

# ---------- 7. 创建工作空间并编译 ----------
build_workspace() {
    info "配置 ROS2 工作空间..."

    # source ROS2 环境
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash

    WS_DIR="${WS_DIR:-$HOME/sentry_ws}"

    if [ -d "$WS_DIR/src" ]; then
        warn "工作空间 $WS_DIR 已存在，跳过创建"
    else
        mkdir -p "$WS_DIR/src"
        info "已创建工作空间: $WS_DIR"
    fi

    # 链接/复制源码（PROJECT_DIR 即为 src/ 目录，直接链接其中的包）。
    # 旧工作空间可能已经有 sentry_nav_bringup，但缺少后续新增包（例如
    # sentry_motion_manager），因此必须逐项补齐，而不是只检查一个包后跳过。
    info "同步源码包到工作空间..."
    for source_path in "$PROJECT_DIR"/*; do
        [ -e "$source_path" ] || continue
        package_name="$(basename "$source_path")"
        target_path="$WS_DIR/src/$package_name"

        if [ -e "$target_path" ] || [ -L "$target_path" ]; then
            continue
        fi

        ln -s "$source_path" "$target_path" 2>/dev/null || \
            cp -rs "$source_path" "$target_path"
    done

    # 安装 ROS 依赖
    info "安装 ROS 包依赖 (rosdep)..."
    cd "$WS_DIR"
    rosdep install -r --from-paths src --ignore-src --rosdistro jazzy -y 2>/dev/null || \
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

# ---------- 8. 配置环境变量 (可选) ----------
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
            echo "source /opt/ros/jazzy/setup.bash"
            echo "$SETUP_LINE"
        } >> ~/.bashrc
        ok "已写入 ~/.bashrc"
    fi
}

# ==================== 主流程 ====================
echo ""
echo "============================================"
echo "  Sentry Nav - 一键环境配置"
echo "  Ubuntu 24.04 + ROS2 Jazzy"
echo "============================================"
echo ""

install_ros2_jazzy
install_gazebo
install_system_deps
install_small_gicp
install_multi_lica_deps
init_rosdep
build_workspace
setup_bashrc

echo ""
echo "============================================"
ok "全部配置完成!"
echo "============================================"
