#!/usr/bin/env python3
# filepath: c:\Desktop\cuoxuanyi\fly1\control\test_camera.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import random

class TestCameraNode(Node):
    def __init__(self):
        super().__init__('test_camera_node')
        
        # 创建发布者，发布桶列表数据
        self.bucket_list_publisher = self.create_publisher(
            Float32MultiArray, '/bucket_list', 10)
        
        # 创建定时器，每0.5秒发布一次模拟数据
        self.timer = self.create_timer(0.5, self.publish_buckets)
        
        self.get_logger().info('测试相机节点已启动')
        
    def publish_buckets(self):
        """发布模拟的桶位置数据"""
        # 创建三个桶的模拟数据
        buckets = [
            [1.0, 0.5, 0.4],  # 桶1 (x, y, size)
            [0.7, 0.2, 0.2],  # 桶2 (最小的桶)
            [0.5, 0.8, 0.6]   # 桶3
        ]
        
        # 添加一些随机噪声
        for bucket in buckets:
            bucket[0] += random.uniform(-0.02, 0.02)
            bucket[1] += random.uniform(-0.02, 0.02)
        
        # 转换为一维列表
        data = []
        for bucket in buckets:
            data.extend(bucket)
            
        # 创建消息
        msg = Float32MultiArray()
        msg.data = data
        
        # 发布消息
        self.bucket_list_publisher.publish(msg)
        self.get_logger().info(f"发布了{len(buckets)}个桶的位置数据")
        
def main(args=None):
    rclpy.init(args=args)
    node = TestCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
