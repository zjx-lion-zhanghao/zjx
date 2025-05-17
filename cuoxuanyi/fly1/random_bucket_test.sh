#!/bin/bash
# 随机桶测试启动脚本 - 加强版

GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== 随机桶数据流测试系统 - 加强版 ===${NC}"

# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 添加执行权限
chmod +x /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/random_bucket_camera.py
chmod +x /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/bucket_filter_node.py
chmod +x /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/data_flow_monitor.py

# 默认参数
CAMERA_MODE="random"
BUCKET_COUNT="3"  # 默认使用3个桶
FIXED_POSITION=""
PUBLISH_RATE=""

# 菜单函数
show_menu() {
  echo -e "${YELLOW}请选择要运行的测试:${NC}"
  echo -e "==== ${BLUE}基础节点操作${NC} ===="
  echo "1) 运行随机桶相机节点"
  echo "2) 运行桶过滤节点"
  echo "3) 运行数据流监控节点"
  echo "4) 运行控制节点"
  echo -e "==== ${BLUE}综合测试${NC} ===="
  echo "5) 同时运行相机和过滤节点"
  echo "6) 同时运行相机、过滤节点和监控节点"
  echo "7) 完整测试 (相机+过滤+控制+监控)"
  echo -e "==== ${BLUE}数据观测${NC} ===="
  echo "8) 查看活跃的ROS2话题"
  echo "9) 监听桶列表话题 (/bucket_list)"
  echo "10) 监听目标位置话题 (/target_position)"
  echo "11) 监听相机状态话题 (/camera_status)"
  echo -e "==== ${BLUE}配置选项${NC} ===="
  echo "12) 配置测试参数"
  echo -e "==== ${BLUE}其他${NC} ===="
  echo "0) 退出"
  echo ""
  
  # 显示当前配置
  echo -e "${BLUE}当前测试参数:${NC}"
  echo "- 相机模式: $CAMERA_MODE"
  if [ "$CAMERA_MODE" = "fixed_count" ]; then
    echo "- 桶数量: $BUCKET_COUNT"
  fi
  if [ "$FIXED_POSITION" = "--fixed-pos" ]; then
    echo "- 使用固定位置: 是"
  else
    echo "- 使用固定位置: 否"
  fi
  echo ""
}

# 运行随机桶相机节点
run_camera() {
  CAMERA_ARGS=""
  if [ "$CAMERA_MODE" = "fixed_count" ]; then
    CAMERA_ARGS="--mode fixed_count --count $BUCKET_COUNT"
  elif [ "$CAMERA_MODE" = "fixed_position" ]; then
    CAMERA_ARGS="--mode fixed_position"
  fi
  
  if [ -n "$FIXED_POSITION" ]; then
    CAMERA_ARGS="$CAMERA_ARGS $FIXED_POSITION"
  fi
  
  if [ -n "$PUBLISH_RATE" ]; then
    CAMERA_ARGS="$CAMERA_ARGS $PUBLISH_RATE"
  fi
  
  echo -e "${GREEN}启动随机桶相机节点...${NC}"
  echo -e "${YELLOW}参数: $CAMERA_ARGS${NC}"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/random_bucket_camera.py $CAMERA_ARGS
}

# 运行桶过滤节点
run_filter() {
  echo -e "${GREEN}启动桶过滤节点...${NC}"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/bucket_filter_node.py
}

# 运行数据流监控节点
run_monitor() {
  echo -e "${GREEN}启动数据流监控节点...${NC}"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/data_flow_monitor.py
}

# 运行控制节点
run_control() {
  echo -e "${GREEN}启动控制节点...${NC}"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/control_v1.py
}

# 同时运行相机和过滤节点
run_camera_and_filter() {
  echo -e "${GREEN}同时启动随机桶相机和过滤节点...${NC}"
  echo "桶过滤节点日志将写入filter.log"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/bucket_filter_node.py > filter.log 2>&1 &
  FILTER_PID=$!
  sleep 2
  
  echo "启动随机桶相机节点(前台)..."
  CAMERA_ARGS=""
  if [ "$CAMERA_MODE" = "fixed_count" ]; then
    CAMERA_ARGS="--mode fixed_count --count $BUCKET_COUNT"
  elif [ "$CAMERA_MODE" = "fixed_position" ]; then
    CAMERA_ARGS="--mode fixed_position"
  fi
  
  if [ -n "$FIXED_POSITION" ]; then
    CAMERA_ARGS="$CAMERA_ARGS $FIXED_POSITION"
  fi
  
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/random_bucket_camera.py $CAMERA_ARGS
  
  # 清理
  echo "清理进程..."
  kill $FILTER_PID 2>/dev/null
}

# 同时运行相机、过滤和监控节点
run_camera_filter_monitor() {
  echo -e "${GREEN}同时启动相机、过滤和监控节点...${NC}"
  echo "桶过滤节点日志将写入filter.log"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/bucket_filter_node.py > filter.log 2>&1 &
  FILTER_PID=$!
  sleep 1
  
  echo "数据流监控节点日志将写入monitor.log"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/data_flow_monitor.py > monitor.log 2>&1 &
  MONITOR_PID=$!
  sleep 2
  
  echo "启动随机桶相机节点(前台)..."
  CAMERA_ARGS=""
  if [ "$CAMERA_MODE" = "fixed_count" ]; then
    CAMERA_ARGS="--mode fixed_count --count $BUCKET_COUNT"
  elif [ "$CAMERA_MODE" = "fixed_position" ]; then
    CAMERA_ARGS="--mode fixed_position"
  fi
  
  if [ -n "$FIXED_POSITION" ]; then
    CAMERA_ARGS="$CAMERA_ARGS $FIXED_POSITION"
  fi
  
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/random_bucket_camera.py $CAMERA_ARGS
  
  # 清理
  echo "清理进程..."
  kill $FILTER_PID $MONITOR_PID 2>/dev/null
}

# 同时运行相机、过滤节点、监控和控制节点 (完整测试)
run_full_test() {
  echo -e "${GREEN}启动完整测试 (相机、过滤器、监控和控制节点)...${NC}"
  echo "桶过滤节点日志将写入filter.log"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/bucket_filter_node.py > filter.log 2>&1 &
  FILTER_PID=$!
  sleep 1
  
  echo "控制节点日志将写入control.log"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/control_v1.py > control.log 2>&1 &
  CONTROL_PID=$!
  sleep 1
  
  echo "数据流监控节点日志将写入monitor.log"
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/data_flow_monitor.py > monitor.log 2>&1 &
  MONITOR_PID=$!
  sleep 2
  
  echo "启动随机桶相机节点(前台)..."
  CAMERA_ARGS=""
  if [ "$CAMERA_MODE" = "fixed_count" ]; then
    CAMERA_ARGS="--mode fixed_count --count $BUCKET_COUNT"
  elif [ "$CAMERA_MODE" = "fixed_position" ]; then
    CAMERA_ARGS="--mode fixed_position"
  fi
  
  if [ -n "$FIXED_POSITION" ]; then
    CAMERA_ARGS="$CAMERA_ARGS $FIXED_POSITION"
  fi
  
  python3 /root/ros2_ws/src/zjx/cuoxuanyi/fly1/control/random_bucket_camera.py $CAMERA_ARGS
  
  # 清理
  echo "清理进程..."
  kill $FILTER_PID $CONTROL_PID $MONITOR_PID 2>/dev/null
}

# 查看活跃话题
show_topics() {
  echo -e "${YELLOW}当前活跃的ROS2话题:${NC}"
  ros2 topic list
  echo ""
  ros2 topic info /bucket_list 2>/dev/null || echo "/bucket_list 话题不活跃"
  echo ""
  ros2 topic info /target_position 2>/dev/null || echo "/target_position 话题不活跃"
  echo ""
  ros2 topic info /camera_status 2>/dev/null || echo "/camera_status 话题不活跃"
}

# 监听桶列表话题
listen_buckets() {
  echo -e "${YELLOW}监听 /bucket_list 话题 (按Ctrl+C退出)...${NC}"
  ros2 topic echo /bucket_list
}

# 监听目标位置话题
listen_target() {
  echo -e "${YELLOW}监听 /target_position 话题 (按Ctrl+C退出)...${NC}"
  ros2 topic echo /target_position
}

# 监听相机状态话题
listen_camera_status() {
  echo -e "${YELLOW}监听 /camera_status 话题 (按Ctrl+C退出)...${NC}"
  ros2 topic echo /camera_status
}

# 配置测试参数
configure_params() {
  echo -e "${BLUE}配置测试参数${NC}"
  echo "选择相机模式:"
  echo "1) 完全随机 (random)"
  echo "2) 固定数量 (fixed_count)"
  echo "3) 固定位置 (fixed_position)"
  read -p "请选择 (1-3): " mode_choice
  
  case $mode_choice in
    1) CAMERA_MODE="random" ;;
    2) 
      CAMERA_MODE="fixed_count"
      echo "桶数量 (最多3个):"
      echo "1) 1个桶"
      echo "2) 2个桶"
      echo "3) 3个桶"
      read -p "请选择 (1-3): " count_choice
      case $count_choice in
        1) BUCKET_COUNT=1 ;;
        2) BUCKET_COUNT=2 ;;
        3) BUCKET_COUNT=3 ;;
        *) 
          echo "无效选择，使用默认值(3个桶)"
          BUCKET_COUNT=3
          ;;
      esac
      ;;
    3) CAMERA_MODE="fixed_position" ;;
    *) echo "无效选择，保持当前设置" ;;
  esac
  
  read -p "使用固定位置？ (y/n): " fixed_pos
  if [[ $fixed_pos == "y" || $fixed_pos == "Y" ]]; then
    FIXED_POSITION="--fixed-pos"
  else
    FIXED_POSITION=""
  fi
  
  read -p "设置发布频率 (Hz, 默认0.5): " publish_rate
  if [[ -n "$publish_rate" ]]; then
    PUBLISH_RATE="--rate $publish_rate"
  else
    PUBLISH_RATE=""
  fi
  
  echo -e "${GREEN}参数已更新:${NC}"
  echo "- 模式: $CAMERA_MODE"
  echo "- 桶数量: ${BUCKET_COUNT:-'随机(最多3个)'}"
  echo "- 固定位置: ${FIXED_POSITION:='否'}"
  echo "- 发布频率: ${PUBLISH_RATE:='默认(0.5Hz)'}"
}

# 主循环
while true; do
  show_menu
  read -p "请输入选项(0-12): " choice
  
  case $choice in
    1) run_camera ;;
    2) run_filter ;;
    3) run_monitor ;;
    4) run_control ;;
    5) run_camera_and_filter ;;
    6) run_camera_filter_monitor ;;
    7) run_full_test ;;
    8) show_topics ;;
    9) listen_buckets ;;
    10) listen_target ;;
    11) listen_camera_status ;;
    12) configure_params ;;
    0) echo "退出测试"; exit 0 ;;
    *) echo -e "${RED}无效选项，请重试${NC}" ;;
  esac
  
  # 操作完成后暂停
  echo ""
  read -p "按Enter键继续..."
  clear
done
