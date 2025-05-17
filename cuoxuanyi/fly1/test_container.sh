#!/bin/bash
# 测试深度相机数据流程的启动脚本（适用于容器环境）

echo "=== 开始测试深度相机到控制节点的数据流程 ==="

# 确保脚本可以在工作空间的任意位置运行
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

# 检查包名和构建问题
cd $WORKSPACE_DIR
echo "当前工作目录: $(pwd)"
echo "包目录结构:"
ls -l src/zjx/cuoxuanyi/fly1

# 设置ROS2环境
echo "设置ROS2环境..."
source /opt/ros/humble/setup.bash

# 确保脚本可执行
chmod +x $SCRIPT_DIR/control/test_deep_camera_flow.py
chmod +x $SCRIPT_DIR/control/bucket_filter_node.py
chmod +x $SCRIPT_DIR/control/control_v1.py
chmod +x $SCRIPT_DIR/control/test_camera.py

# 确保包可发现
pushd src/zjx/cuoxuanyi
ln -sf $(pwd)/fly1/control control
popd

# 包检查
echo "检查包设置..."
cat src/zjx/cuoxuanyi/fly1/setup.py
echo ""
cat src/zjx/cuoxuanyi/fly1/package.xml

# 运行单进程测试（无需gnome-terminal）
echo -e "\n=== 开始单进程测试 ==="

# 启动桶过滤节点
echo "启动模拟相机节点（在后台）..."
python3 $SCRIPT_DIR/control/test_camera.py > camera_node.log 2>&1 &
CAMERA_PID=$!
echo "相机节点PID: $CAMERA_PID"
sleep 2

echo "启动桶过滤节点（在后台）..."
python3 $SCRIPT_DIR/control/bucket_filter_node.py > bucket_filter_node.log 2>&1 &
FILTER_PID=$!
echo "桶过滤节点PID: $FILTER_PID"
sleep 2

echo "启动测试监控节点（在前台，按Ctrl+C终止测试）..."
echo "其他节点的输出被重定向到日志文件，请在测试后查看"
python3 $SCRIPT_DIR/control/test_deep_camera_flow.py

# 清理测试进程
echo "终止测试节点..."
kill $CAMERA_PID $FILTER_PID 2>/dev/null

echo "=== 测试结束 ==="
echo "查看日志文件了解更多信息:"
echo "- camera_node.log: 相机节点的输出"
echo "- bucket_filter_node.log: 桶过滤节点的输出"
