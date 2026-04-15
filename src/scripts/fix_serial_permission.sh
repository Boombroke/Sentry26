#!/bin/bash
set -e
GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
info() { echo -e "${CYAN}[INFO]${NC} $1"; }
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }

USER_NAME=$(whoami)

if groups "$USER_NAME" | grep -q '\bdialout\b'; then
    ok "用户 $USER_NAME 已在 dialout 组"
else
    info "将用户 $USER_NAME 加入 dialout 组（串口访问权限）..."
    sudo usermod -aG dialout "$USER_NAME"
    ok "已加入 dialout 组，需要重新登录生效"
fi

DEVICES=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
if [ -z "$DEVICES" ]; then
    info "未检测到串口设备，请检查 USB 连接"
else
    for dev in $DEVICES; do
        sudo chmod 666 "$dev"
        ok "已设置权限: $dev"
    done
fi
