#!/bin/bash
# 独立测试启动脚本 - 无需ROS2包管理系统

echo "=== 深度相机数据流独立测试 ==="

# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 添加执行权限
chmod +x direct_test.py
chmod +x control/bucket_filter_node.py

# 定义颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# 检查ROS2环境
echo -e "${YELLOW}检查ROS2环境...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误: ROS2命令不可用。请确保已正确设置ROS2环境${NC}"
    exit 1
fi

# 运行测试
echo -e "${GREEN}启动测试...${NC}"
echo "1. 在一个终端启动桶过滤节点"
echo "2. 在另一个终端启动测试脚本"
echo ""

# 询问如何启动
read -p "您想开始哪个组件? [1=桶过滤节点, 2=测试脚本, 3=两者都运行]: " choice

case $choice in
    1)
        echo -e "${GREEN}启动桶过滤节点...${NC}"
        python3 control/bucket_filter_node.py
        ;;
    2)
        echo -e "${GREEN}启动测试脚本...${NC}"
        python3 direct_test.py
        ;;
    3)
        echo -e "${GREEN}同时启动两个组件...${NC}"
        echo "桶过滤节点日志将写入filter.log"
        python3 control/bucket_filter_node.py > filter.log 2>&1 &
        FILTER_PID=$!
        sleep 2
        
        echo "启动测试脚本(前台)..."
        python3 direct_test.py
        
        # 清理
        echo "清理进程..."
        kill $FILTER_PID 2>/dev/null
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac

echo -e "${GREEN}测试完成${NC}"
