#!/bin/bash
# 最简单的深度相机到控制测试脚本

echo "=== 开始最简化测试 ==="

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置环境
source /opt/ros/humble/setup.bash
export PYTHONPATH=$SCRIPT_DIR:$PYTHONPATH

# 显示路径信息
echo "脚本目录: $SCRIPT_DIR"
echo "Python路径: $PYTHONPATH"
echo "当前目录: $(pwd)"

# 提供函数来直接运行Python节点
runTest() {
    echo "启动测试相机数据流..."
    # 因为我们没有使用ROS2的包管理，直接运行Python脚本
    
    echo "启动测试相机节点 (发布桶数据)..."
    python3 $SCRIPT_DIR/control/test_camera.py &
    CAMERA_PID=$!
    sleep 2
    
    echo "启动桶过滤节点 (筛选最小桶)..."
    python3 $SCRIPT_DIR/control/bucket_filter_node.py &
    FILTER_PID=$!
    sleep 2
    
    echo "启动控制节点 (接收目标位置)..."
    python3 $SCRIPT_DIR/control/test_control_node.py &
    CONTROL_TEST_PID=$!
    sleep 2
    
    echo "启动数据监控节点..."
    python3 $SCRIPT_DIR/control/test_deep_camera_flow.py
    
    # 清理
    kill $CAMERA_PID $FILTER_PID $CONTROL_TEST_PID 2>/dev/null
}

# 显示可用话题
checkTopics() {
    echo "=== 当前活动话题 ==="
    ros2 topic list
    
    echo -e "\n=== 话题详情 ==="
    ros2 topic info /bucket_list 2>/dev/null || echo "/bucket_list 话题不可用"
    ros2 topic info /target_position 2>/dev/null || echo "/target_position 话题不可用"
}

# 显示菜单
echo "选择要执行的操作:"
echo "1) 运行测试流程"
echo "2) 检查ROS2话题"
echo "3) 退出"

read -p "请输入选项 (1-3): " choice

case $choice in
    1) runTest ;;
    2) checkTopics ;;
    *) echo "退出测试" ;;
esac
