#!/bin/bash
# Install Python deps for serial_visualizer.py (pyqtgraph + PyQt5).
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

info() { echo -e "${CYAN}[INFO]${NC} $1"; }
ok() { echo -e "${GREEN}[OK]${NC} $1"; }
err() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

if python3 -c "import pyqtgraph" 2>/dev/null; then
  ok "pyqtgraph 已安装"
  exit 0
fi

info "安装 pyqtgraph（优先 apt，失败则 pip）..."

if command -v apt-get &>/dev/null; then
  sudo apt-get update -qq
  if sudo apt-get install -y python3-pyqtgraph python3-pyqt5 2>/dev/null; then
    python3 -c "import pyqtgraph" || err "apt 安装后仍无法 import pyqtgraph"
    ok "已通过 apt 安装 python3-pyqtgraph"
    exit 0
  fi
fi

pip3 install --user pyqtgraph PyQt5 --break-system-packages \
  || pip3 install --user pyqtgraph PyQt5 \
  || err "pip 安装失败，请检查网络后重试"

python3 -c "import pyqtgraph" || err "pip 安装后仍无法 import pyqtgraph"
ok "已通过 pip 安装 pyqtgraph"
