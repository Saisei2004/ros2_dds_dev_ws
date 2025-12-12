#!/bin/bash

# ROS2 Jazzyワークスペース セットアップスクリプト
# 使用方法: source setup_workspace.sh

echo "ROS2 Jazzyワークスペースをセットアップしています..."

# ROS2 Jazzyの環境をソース
source /opt/ros/jazzy/setup.bash

# ワークスペースをソース
source install/setup.bash

echo "ワークスペースがセットアップされました！"
echo "現在のROS_DISTRO: $ROS_DISTRO"
echo "ワークスペースディレクトリ: $(pwd)"

# パッケージリストを表示
echo ""
echo "インストール済みパッケージ:"
ros2 pkg list | grep -v "/opt/ros/jazzy" | head -10
