#!/bin/bash
# WSL 构建包装脚本:把 Windows 侧源码同步到 WSL 原生文件系统构建,避免 /mnt/c IO 慢。
# 用法:
#   tools/wslbuild.sh build          # colcon build
#   tools/wslbuild.sh test [PKG]     # colcon test(可指定包)
#   tools/wslbuild.sh run <cmd...>   # 在已 source 的环境中执行任意命令
set -e

SRC_WIN=/mnt/c/Users/ASUS/Desktop/ros2/nav_ws
WS=~/nav_ws
MODE="$1"; shift || true

source /opt/ros/humble/setup.bash

mkdir -p "$WS"
# 同步源码(删除多余文件,排除构建产物)
rsync -a --delete "$SRC_WIN/src/" "$WS/src/"

cd "$WS"

case "$MODE" in
  build)
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
    ;;
  test)
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    if [ -n "$1" ]; then
      colcon test --packages-select "$@" --event-handlers console_direct+ --return-code-on-test-failure
    else
      colcon test --event-handlers console_direct+ --return-code-on-test-failure
    fi
    colcon test-result --verbose || true
    ;;
  run)
    source "$WS/install/setup.bash" 2>/dev/null || true
    "$@"
    ;;
  *)
    echo "usage: wslbuild.sh {build|test|run} ..." >&2
    exit 2
    ;;
esac
