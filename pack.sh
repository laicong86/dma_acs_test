#!/bin/sh
# 工程快照打包：每次改完代码跑一次，产物放在工程目录的上一级。
#
#   ./pack.sh  ->  ../dma_acs_test_20261103-102635.tar
#
# 只打源码/文档/构建脚本；.git、编译产物、调试与运行时残留都不进包。
set -eu

here=$(cd "$(dirname "$0")" && pwd)
out="$(dirname "$here")/dma_acs_test_$(date +%Y%m%d-%H%M%S).tar"

tar -C "$here" \
    --exclude=./.git \
    --exclude=./dma_acs_test \
    --exclude=./dma_acs_test_v2 \
    --exclude=./debug.log \
    --exclude='*.o' \
    --exclude='*.rec' \
    -cf "$out" .

echo "$out"
