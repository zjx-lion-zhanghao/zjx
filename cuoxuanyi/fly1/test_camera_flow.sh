#!/bin/bash
# 测试深度相机数据流程的启动脚本

echo "=== 开始测试深度相机到控制节点的数据流程 ==="

# 确保脚本可以在工作空间的任意位置运行
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

# 创建源文件链接以确保ROS2能找到包
cd $WORKSPACE_DIR
if [ ! -L "src/fly1" ]; then
  echo "创建符号链接，使ROS2能找到包..."
  ln -s src/zjx/cuoxuanyi/fly1 src/fly1
fi

# 设置ROS2环境
echo "设置ROS2环境..."
source /opt/ros/humble/setup.bash

# 构建包
echo "构建工作空间..."
colcon build --packages-select fly1 --symlink-install

# 设置构建后的环境
source install/setup.bash

# 确保脚本可执行
chmod +x $SCRIPT_DIR/control/test_deep_camera_flow.py
chmod +x $SCRIPT_DIR/control/bucket_filter_node.py
chmod +x $SCRIPT_DIR/control/control_v1.py

# 启动测试
echo "=== 启动测试节点 ==="
echo "1. 启动桶过滤节点..."
gnome-terminal -- bash -c "ros2 run fly1 bucket_filter_node.py; exec bash" &
sleep 1

echo "2. 启动测试流程节点..."
gnome-terminal -- bash -c "ros2 run fly1 test_deep_camera_flow.py; exec bash" &
sleep 1

echo "3. 启动ROS2话题监控..."
gnome-terminal -- bash -c "ros2 topic echo /bucket_list; exec bash" &
gnome-terminal -- bash -c "ros2 topic echo /target_position; exec bash" &

echo "=== 测试已启动 ==="
echo "查看各终端窗口输出以检查测试结果"
echo "按Ctrl+C终止测试"

# 显示节点和话题信息
sleep 3
echo ""
echo "=== 活动节点列表 ==="
ros2 node list
echo ""
echo "=== 活动话题列表 ==="
ros2 topic list
echo ""

# 等待用户终止
read -p "按Enter键终止测试..."
