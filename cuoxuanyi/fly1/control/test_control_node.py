#!/usr/bin/env python3
"""
测试脚本 - 验证控制节点接收和处理目标位置信息
此脚本模拟bucket_filter_node发布目标位置，然后验证control_v1是否正确接收
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class TestControlNode(Node):
    """测试控制节点接收目标位置的节点"""
    
    def __init__(self):
        super().__init__('test_control_node')
        
        # 创建目标位置发布者
        self.target_position_pub = self.create_publisher(
            Point,
            '/target_position',
            10
        )
        
        # 创建测试序列定时器
        self.timer = self.create_timer(2.0, self.run_test_sequence)
        self.test_count = 0
        self.test_positions = [
            (0.5, 0.3, 0.0),   # 测试点1
            (-0.2, 0.1, 0.0),  # 测试点2
            (0.7, -0.4, 0.0)   # 测试点3
        ]
        
        self.get_logger().info('控制节点测试已启动，将发送模拟目标位置')
        
    def run_test_sequence(self):
        """运行测试序列，发布一系列目标位置"""
        if self.test_count < len(self.test_positions):
            x, y, z = self.test_positions[self.test_count]
            
            # 创建并发布目标位置
            msg = Point()
            msg.x = x
            msg.y = y
            msg.z = z
            
            self.target_position_pub.publish(msg)
            self.get_logger().info(f'测试 {self.test_count+1}/{len(self.test_positions)}: 已发布目标位置 [{x}, {y}, {z}]')
            
            # 提供验证指引
            self.get_logger().info(f'查看control_v1节点日志，确认是否收到并处理了此目标位置')
            
            self.test_count += 1
        else:
            self.get_logger().info('=== 测试序列完成 ===')
            self.get_logger().info('请检查control_v1节点是否正确接收并处理了所有目标位置')
            self.get_logger().info('注意：如果control_v1处于执行任务流程中，它可能会根据自身状态决定是否使用接收到的目标位置')
            
            # 停止定时器
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TestControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断测试')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
