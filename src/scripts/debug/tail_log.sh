#!/bin/bash
# tail_log.sh - 实时跟随最新一次 ros2 launch 的日志
# 用法：
#   bash src/scripts/debug/tail_log.sh          # 跟随最新 launch.log
#   bash src/scripts/debug/tail_log.sh <node>   # 跟随指定节点日志（如 controller_server）

set -e

LATEST=$(ls -td ~/.ros/log/*/ 2>/dev/null | head -1)
if [ -z "$LATEST" ]; then
    echo "No launch logs found in ~/.ros/log/"
    exit 1
fi

echo "最新 launch 目录: $LATEST"

if [ -z "$1" ]; then
    # 默认 tail launch.log
    LOG="${LATEST}launch.log"
    if [ -f "$LOG" ]; then
        echo "跟随: $LOG"
        tail -F "$LOG"
    else
        echo "launch.log 不存在，列出当前目录文件："
        ls -la "$LATEST"
    fi
else
    # 查找匹配节点的日志
    NODE=$1
    MATCH=$(ls "$LATEST" 2>/dev/null | grep -i "$NODE" | head -5)
    if [ -z "$MATCH" ]; then
        echo "未找到匹配 '$NODE' 的日志文件，当前目录有："
        ls "$LATEST" | head -20
        exit 1
    fi
    echo "匹配文件："
    echo "$MATCH"
    echo ""
    FIRST=$(echo "$MATCH" | head -1)
    echo "跟随第一个: ${LATEST}${FIRST}"
    tail -F "${LATEST}${FIRST}"
fi
