#!/bin/bash
# 测试深度相机->桶过滤->控制节点的流程

# 确保ROS2环境已经设置
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 构建项目
cd ~/ros2_ws
colcon build --packages-select fly1

# 设置环境
source ~/ros2_ws/install/setup.bash

# 创建 Screen 会话运行各个节点
echo "启动测试环境..."

# 终端1: 运行ROS2 核心
gnome-terminal -- bash -c "ros2 run rqt_graph rqt_graph; exec bash"

# 终端2: 运行模拟相机节点
gnome-terminal -- bash -c "ros2 run fly1 test_camera.py; exec bash"

# 终端3: 运行桶过滤节点
gnome-terminal -- bash -c "ros2 run fly1 bucket_filter_node.py; exec bash"

# 终端4: 运行监控节点
gnome-terminal -- bash -c "ros2 run fly1 test_bucket_detection.py; exec bash"

# 等待节点启动
echo "等待节点启动..."
sleep 2

# 查看主题列表
echo "当前活动的话题列表："
ros2 topic list

# 监控桶列表话题
echo "监控 /bucket_list 话题的数据："
gnome-terminal -- bash -c "ros2 topic echo /bucket_list; exec bash"

# 监控目标位置话题
echo "监控 /target_position 话题的数据："
gnome-terminal -- bash -c "ros2 topic echo /target_position; exec bash"

echo "测试环境已启动"
