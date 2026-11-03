#!/bin/sh
# 工程快照打包：每次改完代码跑一次，产物放在工程目录的上一级。
#
#   ./pack.sh          ->  ../dma_acs_test_20261103-102746.tar，并清掉更早的包
#   ./pack.sh 3        ->  保留最近 3 份
#
# 只打源码/文档/构建脚本；.git、编译产物、调试与运行时残留都不进包。
set -eu

keep=${1:-1}
case "$keep" in
    ''|*[!0-9]*) echo "用法: $0 [保留份数，默认 1]" >&2; exit 2 ;;
esac
[ "$keep" -ge 1 ] || { echo "保留份数至少为 1" >&2; exit 2; }

here=$(cd "$(dirname "$0")" && pwd)
dir=$(dirname "$here")
out="$dir/dma_acs_test_$(date +%Y%m%d-%H%M%S).tar"

tar -C "$here" \
    --exclude=./.git \
    --exclude=./dma_acs_test \
    --exclude=./dma_acs_test_v2 \
    --exclude=./debug.log \
    --exclude='*.o' \
    --exclude='*.rec' \
    -cf "$out" .

echo "$out"

# 清理历史包：按修改时间保留最新 $keep 份（新打的这份永远在最前），其余删除。
# 只清这个命名模式，别的文件一概不碰。
old=$(ls -1t "$dir"/dma_acs_test*.tar 2>/dev/null | tail -n "+$((keep + 1))" || true)
if [ -n "$old" ]; then
    printf '%s\n' "$old" | while IFS= read -r f; do
        [ -n "$f" ] && [ -f "$f" ] || continue
        rm -f -- "$f" && echo "已清理旧包: $(basename "$f")"
    done
fi
