#!/bin/bash
# 修复 point_lio / small_gicp 启动报错:
#   libpcl_io.so: undefined symbol: libusb_set_option
# 原因: 系统 libusb 过旧，或 OpenVINO 等环境把旧版 libusb 插到 LD_LIBRARY_PATH 前面

set -e
CYAN='\033[0;36m'; GREEN='\033[0;32m'; NC='\033[0m'
info() { echo -e "${CYAN}[INFO]${NC} $1"; }
ok()   { echo -e "${GREEN}[OK]${NC} $1"; }

info "安装/重装 libusb..."
sudo apt update
sudo apt install -y --reinstall libusb-1.0-0 libusb-1.0-0-dev

LIBUSB="$(ldconfig -p | grep 'libusb-1.0.so.0' | head -1 | awk '{print $NF}')"
if [[ -z "$LIBUSB" ]]; then
  echo "未找到 libusb-1.0.so.0"
  exit 1
fi
ok "libusb: $LIBUSB"

if ! nm -D "$LIBUSB" 2>/dev/null | grep -q 'libusb_set_option'; then
  echo "当前 libusb 仍无 libusb_set_option，请升级系统或检查第三方 LD_LIBRARY_PATH"
  exit 1
fi
ok "libusb_set_option 符号存在"

info "常见原因: 海康 MVS (/opt/MVS/lib/64/libusb) 或 OpenVINO 把旧 libusb 插到 LD_LIBRARY_PATH 前面"
info "自启脚本应在 source ROS 后 prepend: LD_LIBRARY_PATH=/lib/...:\$LD_LIBRARY_PATH（勿覆盖，否则找不到 librcl_action.so）"
info "GNOME 自启请用 bash --noprofile，不要用 bash -lc（会读 ~/.bashrc）"
info "验证: LD_LIBRARY_PATH=/lib/x86_64-linux-gnu ldd install/point_lio/.../pointlio_mapping | grep libusb"
info "应显示 /lib/x86_64-linux-gnu/libusb-1.0.so.0，而不是 /opt/MVS/..."
