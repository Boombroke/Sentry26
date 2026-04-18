#!/bin/bash
# archive_log.sh - 把最新一次 launch 的日志归档到项目 logs/ 目录（带时间戳）
# 方便 debug 时对照历次运行记录。
# 用法：
#   bash src/scripts/debug/archive_log.sh              # 默认标签 "run"
#   bash src/scripts/debug/archive_log.sh nav_test     # 自定义标签

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ARCHIVE_DIR="$PROJECT_ROOT/logs"
mkdir -p "$ARCHIVE_DIR"

LABEL="${1:-run}"
LATEST=$(ls -td ~/.ros/log/*/ 2>/dev/null | head -1)
if [ -z "$LATEST" ]; then
    echo "No launch logs found in ~/.ros/log/"
    exit 1
fi

TIMESTAMP=$(date +%Y%m%d-%H%M%S)
TARGET="$ARCHIVE_DIR/${TIMESTAMP}-${LABEL}"

cp -r "$LATEST" "$TARGET"
echo "已归档: $TARGET"
echo ""
echo "包含文件："
ls "$TARGET" | head -20
echo ""
echo "=== 归档列表 ==="
ls -lat "$ARCHIVE_DIR" | head -10
