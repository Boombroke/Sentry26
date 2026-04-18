#!/bin/bash
# clean_logs.sh - 清理项目 logs/ 目录的旧 bag
# 用法:
#   bash src/scripts/debug/clean_logs.sh           # 列出所有 bag + 大小
#   bash src/scripts/debug/clean_logs.sh --keep 3  # 保留最新 3 个, 删其余
#   bash src/scripts/debug/clean_logs.sh --all     # 全删

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOGS_DIR="$PROJECT_ROOT/logs"

if [ ! -d "$LOGS_DIR" ]; then
    echo "logs/ 目录不存在"
    exit 0
fi

case "${1:-list}" in
    list|--list|"")
        echo "=== $LOGS_DIR 内容 ==="
        ls -la "$LOGS_DIR"
        echo ""
        echo "=== 每个 bag 大小 ==="
        du -sh "$LOGS_DIR"/*/ 2>/dev/null || echo "(无 bag)"
        echo ""
        echo "=== 总大小 ==="
        du -sh "$LOGS_DIR" 2>/dev/null
        echo ""
        echo "用法:"
        echo "  $0 --keep 3    # 保留最新 3 个"
        echo "  $0 --all       # 全删"
        ;;
    --keep)
        KEEP="${2:-3}"
        cd "$LOGS_DIR"
        TO_DELETE=$(ls -td 2026*/ 2>/dev/null | tail -n +$((KEEP + 1)))
        if [ -z "$TO_DELETE" ]; then
            echo "无需删除（现有 bag 数 <= $KEEP）"
        else
            echo "将删除以下 bag (保留最新 $KEEP 个):"
            echo "$TO_DELETE"
            read -p "确认? [y/N] " YN
            if [ "$YN" = "y" ] || [ "$YN" = "Y" ]; then
                echo "$TO_DELETE" | xargs -r rm -rf
                echo "已删除"
            fi
        fi
        ;;
    --all)
        echo "将删除所有 bag:"
        ls -d "$LOGS_DIR"/2026*/ 2>/dev/null
        read -p "确认? [y/N] " YN
        if [ "$YN" = "y" ] || [ "$YN" = "Y" ]; then
            rm -rf "$LOGS_DIR"/2026*/
            echo "已删除"
        fi
        ;;
    *)
        echo "未知参数: $1"
        echo "用法: $0 [list|--keep N|--all]"
        exit 1
        ;;
esac
