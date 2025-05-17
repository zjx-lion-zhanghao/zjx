#!/usr/bin/env python3
"""
直接测试脚本 - 测试深度相机数据处理流程
此脚本直接发布和订阅相关话题，无需ROS2包依赖
"""

import sys
import time
import os

# 确保我们可以导入ROS2模块
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    from geometry_msgs.msg import Point
except ImportError as e:
    print(f"错误: 无法导入ROS2模块: {e}")
    print("请确保已正确设置ROS2环境:")
    print("  source /opt/ros/humble/setup.bash")
    sys.exit(1)

class DirectTest(Node):
    """一体化测试节点，同时模拟相机和监控数据流"""
    
    def __init__(self):
        super().__init__('direct_test_node')
        
        # 创建桶列表发布者（模拟深度相机）
        self.bucket_list_pub = self.create_publisher(
            Float32MultiArray,
            '/bucket_list',
            10
        )
        
        # 监听目标位置（被处理后的最小桶位置）
        self.target_position_sub = self.create_subscription(
            Point,
            '/target_position',
            self.target_position_callback,
            10
        )
        
        # 记录测试状态
        self.received_target = False
        self.test_passed = False
        self.target_position = None
        self.test_count = 0
        self.max_tests = 5
        self.expected_x = -0.2
        self.expected_y = 0.1
        
        # 创建定时器发布测试数据
        self.publish_timer = self.create_timer(2.0, self.publish_test_data)
        self.evaluation_timer = self.create_timer(10.0, self.evaluate_test)
        
        self.get_logger().info('直接测试节点已启动 - 模拟相机并监控数据流')
        
    def publish_test_data(self):
        """发布测试用的桶数据"""
        if self.test_count >= self.max_tests:
            return
            
        self.test_count += 1
        
        # 创建带有明确大小差异的测试数据
        # 格式：[x1, y1, size1, x2, y2, size2, ...]
        # 其中第二个桶是最小的
        test_data = [
            0.5, 0.3, 0.8,  # 桶1: 大尺寸
            -0.2, 0.1, 0.2,  # 桶2: 最小的桶
            0.7, -0.4, 0.6   # 桶3: 中等尺寸
        ]
        
        msg = Float32MultiArray()
        msg.data = test_data
        
        self.bucket_list_pub.publish(msg)
        self.get_logger().info(f'测试 {self.test_count}/{self.max_tests}: 已发布桶数据')
        self.get_logger().info(f'已发布桶数据: [{len(test_data)//3}个桶]')
        for i in range(len(test_data)//3):
            x = test_data[i*3]
            y = test_data[i*3+1]
            size = test_data[i*3+2]
            self.get_logger().info(f'  桶 {i+1}: x={x:.2f}, y={y:.2f}, size={size:.2f}')
        
    def target_position_callback(self, msg):
        """验证接收到的目标位置是否正确"""
        self.target_position = msg
        self.received_target = True
        
        # 检查与预期值的差异（允许小误差）
        x_diff = abs(msg.x - self.expected_x)
        y_diff = abs(msg.y - self.expected_y)
        
        if x_diff < 0.05 and y_diff < 0.05:
            self.test_passed = True
            self.get_logger().info('✅ 测试成功! 接收到正确的目标位置')
        else:
            self.get_logger().info(f'❌ 位置不匹配! 收到: [{msg.x:.2f}, {msg.y:.2f}], 预期: [{self.expected_x}, {self.expected_y}]')
            
        self.get_logger().info(f'收到目标位置: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')
    
    def evaluate_test(self):
        """评估测试结果"""
        if not self.received_target:
            self.get_logger().error('❌ 测试失败! 未收到任何目标位置消息')
            self.get_logger().info('请检查桶过滤节点是否正常运行')
        else:
            if self.test_passed:
                self.get_logger().info('✅ 测试成功完成! 数据流正常工作')
            else:
                self.get_logger().warning('⚠️ 收到了目标位置，但不是预期的最小桶位置')
                
        # 输出测试总结
        self.get_logger().info('=== 测试总结 ===')
        self.get_logger().info(f'发送测试数据次数: {self.test_count}')
        self.get_logger().info(f'接收到目标位置: {"是" if self.received_target else "否"}')
        self.get_logger().info(f'接收的位置正确: {"是" if self.test_passed else "否"}')
        
        # 提示下一步操作
        self.get_logger().info('')
        self.get_logger().info('继续测试请按Ctrl+C终止当前测试，然后再次运行')
        
        # 暂停定时器，停止发送测试数据
        if self.test_count >= self.max_tests:
            self.publish_timer.cancel()
            self.evaluation_timer.cancel()

def main(args=None):
    # 显示启动信息
    print("=== 启动一体化测试 ===")
    print("此脚本将同时:")
    print("1. 模拟深度相机，发布桶列表数据")
    print("2. 监听目标位置，验证数据处理流程")
    print("")
    print("请确保桶过滤节点(bucket_filter_node.py)已启动")
    print("如果要测试完整流程，请同时启动控制节点(control_v1.py)")
    print("")
    
    try:
        rclpy.init(args=args)
        node = DirectTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        print("测试结束")

if __name__ == '__main__':
    main()
