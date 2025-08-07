#!/bin/bash -e

# 設定 ROS_MASTER_URI
if [ $# = 1 ]; then
    export ROS_MASTER_URI=$1
else
    export ROS_MASTER_URI=http://localhost:11311
fi

# Source 環境
source /ros2_humble/install/setup.bash
source /ros2_humble/install/ros1_bridge/share/ros1_bridge/local_setup.bash

# 如果 bridge node 已經存在就不再啟動
if ros2 node list | grep -q "/ros_bridge_main"; then
    echo "🟡 [ros_bridge_main] is already running, skip starting new bridge."
    exit 0
fi

# 執行 dynamic_bridge 並使用不會撞名的 node 名
echo "✅ Starting ros1_bridge as [/ros_bridge_main]..."
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args --remap __node:=ros_bridge_main
