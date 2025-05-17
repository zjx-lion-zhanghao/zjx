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
        """发布模拟的桶位置数据，完全使用随机数"""
        # 随机决定有多少个桶 (2-5个)
        bucket_count = random.randint(2, 5)
        
        # 随机生成桶的数据
        buckets = []
        min_size_index = random.randint(0, bucket_count-1)  # 随机选择一个桶作为最小桶
        
        for i in range(bucket_count):
            # 随机位置，x和y范围在 -1.5 到 1.5 米之间
            x = random.uniform(-1.5, 1.5)
            y = random.uniform(-1.5, 1.5)
            
            # 随机大小，确保有一个桶是明显最小的
            if i == min_size_index:
                size = random.uniform(0.1, 0.3)  # 最小的桶
                self.get_logger().info(f"最小的桶 (索引 {i}): x={x:.2f}, y={y:.2f}, size={size:.2f}")
            else:
                size = random.uniform(0.4, 1.0)  # 其他桶
            
            buckets.append([x, y, size])
        
        # 转换为一维列表
        data = []
        for bucket in buckets:
            data.extend(bucket)
            
        # 创建消息
        msg = Float32MultiArray()
        msg.data = data
        
        # 发布消息
        self.bucket_list_publisher.publish(msg)
        self.get_logger().info(f"发布了{bucket_count}个随机桶的位置和大小数据")
        
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
