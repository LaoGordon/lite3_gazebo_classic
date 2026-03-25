#!/bin/bash

# Gazebo Classic world 启动脚本
# 功能：清理残留进程，加载指定 world，并启动机器人仿真

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
DEFAULT_WORLD="${WORKSPACE_ROOT}/src/lite3_gazebo_classic/libraries/quadruped_playground/worlds/test_world.world"
PLAYGROUND_MODELS="${WORKSPACE_ROOT}/src/lite3_gazebo_classic/libraries/quadruped_playground/models"
WORLD_FILE="${1:-$DEFAULT_WORLD}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Gazebo Classic World 启动脚本${NC}"
echo -e "${GREEN}  工作空间：${WORKSPACE_ROOT}${NC}"
echo -e "${GREEN}  World：${WORLD_FILE}${NC}"
echo -e "${GREEN}========================================${NC}"

if [ ! -f "$WORLD_FILE" ]; then
    echo -e "${RED}错误：world 文件不存在：${WORLD_FILE}${NC}"
    exit 1
fi

echo -e "${YELLOW}[1/5] 清理残留的 Gazebo 进程...${NC}"
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
pkill -9 -x gazebo 2>/dev/null || true
ps aux | grep -E "gazebo|gzserver|gzclient" | grep -v grep | grep -v run_gazebo_world.sh | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 1

REMAINING=$(ps aux | grep -E "(gazebo|gzserver|gzclient)" | grep -v grep | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    echo -e "${RED}警告：仍有 $REMAINING 个 Gazebo 相关进程在运行${NC}"
    ps aux | grep -E "(gazebo|gzserver|gzclient)" | grep -v grep
    echo -e "${YELLOW}尝试使用 fuser 清理端口...${NC}"
    fuser -k 11345/tcp 2>/dev/null || true
    sleep 1
fi

echo -e "${YELLOW}[2/5] 清理 Gazebo 共享内存...${NC}"
rm -f /tmp/gazebo_* 2>/dev/null || true

echo -e "${YELLOW}[3/5] 设置环境变量...${NC}"

if [ -f "/usr/share/gazebo/setup.sh" ]; then
    source /usr/share/gazebo/setup.sh
    echo -e "${GREEN}  ✓ 已加载 Gazebo 环境${NC}"
fi

LIVOX_PLUGIN_PATH="${WORKSPACE_ROOT}/install/ros2_livox_simulation/lib"
if [ -d "$LIVOX_PLUGIN_PATH" ]; then
    export GAZEBO_PLUGIN_PATH="${LIVOX_PLUGIN_PATH}:${GAZEBO_PLUGIN_PATH:-}"
    echo -e "${GREEN}  ✓ GAZEBO_PLUGIN_PATH: ${GAZEBO_PLUGIN_PATH}${NC}"
fi

export GAZEBO_MODEL_PATH="${PLAYGROUND_MODELS}:${GAZEBO_MODEL_PATH:-}"
echo -e "${GREEN}  ✓ GAZEBO_MODEL_PATH: ${GAZEBO_MODEL_PATH}${NC}"

echo -e "${YELLOW}[4/5] 设置其他环境变量...${NC}"
export QT_QPA_PLATFORM=xcb
export GTK_IM_MODULE=
export QT_IM_MODULE=
export XMODIFIERS=
echo -e "${GREEN}  ✓ 已设置 GUI 相关环境变量${NC}"

echo -e "${YELLOW}[5/5] 启动 Gazebo Classic world...${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  启动命令：ros2 launch lite3_description gazebo_classic.launch.py world:=${WORLD_FILE}${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

cd "$WORKSPACE_ROOT"
source install/setup.bash
ros2 launch lite3_description gazebo_classic.launch.py world:="$WORLD_FILE"
