#!/bin/bash

# 同步脚本：将 src/lite3_gazebo_classic 的内容同步到 git_folders
# 用法：./sync_to_git_folders.sh

set -e

echo "开始同步 src/lite3_gazebo_classic 到 git_folders..."

# 确保 git_folders 目录存在
mkdir -p git_folders

# 使用 rsync 同步，排除 .git 目录
rsync -av --delete \
  --exclude='.git' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  --exclude='*.pyc' \
  --exclude='__pycache__/' \
  --exclude='*.o' \
  --exclude='*.so' \
  --exclude='.vscode/' \
  --exclude='.idea/' \
  --exclude='*~' \
  --exclude='*.swp' \
  src/lite3_gazebo_classic/ git_folders/

echo "同步完成！"
echo ""
echo "下一步操作："
echo "1. 检查 git_folders 目录的内容"
echo "2. 运行 git add git_folders/"
echo "3. 运行 git commit -m '更新: 同步最新代码'"
echo "4. 运行 git push origin main"
