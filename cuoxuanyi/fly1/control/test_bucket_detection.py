#!/usr/bin/env python3
"""
测试脚本 - 验证深度相机数据处理流程
用于测试深度相机 -> bucket_filter_node -> control_v1 的数据传输流程
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import sys
import time

class DataMonitorNode(Node):
    """监控测试过程中的数据流动"""
    
    def __init__(self):
        super().__init__('data_monitor_node')
        
        # 监听桶列表数据
        self.bucket_list_sub = self.create_subscription(
            Float32MultiArray,
            '/bucket_list',
            self.bucket_list_callback,
            10
        )
        
        # 监听目标位置数据
        self.target_position_sub = self.create_subscription(
            Point,
            '/target_position',
            self.target_position_callback,
            10
        )
        
        self.get_logger().info('数据监控节点已启动')
        
    def bucket_list_callback(self, msg):
        """记录接收到的桶列表数据"""
        data = msg.data
        bucket_count = len(data) // 3
        
        self.get_logger().info(f'监听到 {bucket_count} 个桶的数据:')
        for i in range(bucket_count):
            x = data[i*3]
            y = data[i*3 + 1]
            size = data[i*3 + 2]
            self.get_logger().info(f'  桶 {i+1}: x={x:.2f}, y={y:.2f}, size={size:.2f}')
    
    def target_position_callback(self, msg):
        """记录目标位置数据"""
        self.get_logger().info(f'监听到目标位置: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DataMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
