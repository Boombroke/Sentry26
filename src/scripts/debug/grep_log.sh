#!/bin/bash
# grep_log.sh - 跨最新 launch 所有节点日志搜索关键字
# 用法：
#   bash src/scripts/debug/grep_log.sh "error"
#   bash src/scripts/debug/grep_log.sh "base_footprint"

set -e

if [ -z "$1" ]; then
    echo "用法: $0 <pattern>"
    echo "示例:"
    echo "  $0 error           # 搜 error 关键字"
    echo "  $0 'imu loop'      # 搜多词（引号包裹）"
    echo "  $0 'TF lookup'     # TF 相关"
    exit 1
fi

LATEST=$(ls -td ~/.ros/log/*/ 2>/dev/null | head -1)
if [ -z "$LATEST" ]; then
    echo "No launch logs found in ~/.ros/log/"
    exit 1
fi

echo "搜索目录: $LATEST"
echo "关键字: $1"
echo "---"
# -r 递归, -n 行号, -I 忽略二进制, --color 高亮
grep -rniI --color=always "$1" "$LATEST" 2>/dev/null | head -100
