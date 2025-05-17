#!/usr/bin/env python3
# filepath: c:\Desktop\cuoxuanyi\fly1\control\bucket_filter_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # 用于接收深度相机发送的桶位置列表
from geometry_msgs.msg import Point  # 用于发布目标位置

class BucketFilterNode(Node):
    """
    接收多个桶的位置和大小数据，识别最小的桶，并将其位置发布为目标位置
    """
    def __init__(self):
        super().__init__('bucket_filter_node')
        
        # 创建订阅者，监听深度相机发布的桶位置列表
        # 假设深度相机发布的是Float32MultiArray类型的消息，包含多个桶的x,y,size数据
        self.bucket_list_subscriber = self.create_subscription(
            Float32MultiArray, 
            '/bucket_list', 
            self.bucket_list_callback, 
            10
        )
        
        # 创建发布者，发布最小桶的位置到target_position话题
        self.target_position_publisher = self.create_publisher(
            Point, 
            '/target_position', 
            10
        )
        
        self.get_logger().info('桶筛选节点已启动，等待接收桶列表数据')
        
    def bucket_list_callback(self, msg):
        """处理接收到的桶列表数据，找到最小桶并发布位置"""
        # 获取数据
        data = msg.data
        
        # 确保数据格式正确
        if len(data) == 0 or len(data) % 3 != 0:
            self.get_logger().warn(f'接收到的数据格式错误，长度:{len(data)}')
            return
            
        # 解析数据：假设每个桶的数据格式为[x, y, size]，连续存储在列表中
        buckets = []
        bucket_count = len(data) // 3
        
        for i in range(bucket_count):
            bucket_x = data[i*3]
            bucket_y = data[i*3 + 1]
            bucket_size = data[i*3 + 2]  # 桶的大小（可能是直径、面积等）
            buckets.append((bucket_x, bucket_y, bucket_size))
            
        self.get_logger().info(f'接收到{len(buckets)}个桶的数据')
        
        # 找出最小的桶（基于size值）
        if not buckets:
            self.get_logger().warn('没有检测到任何桶')
            return
            
        # 按桶的大小排序，取最小的
        smallest_bucket = min(buckets, key=lambda x: x[2])
        smallest_x = smallest_bucket[0]
        smallest_y = smallest_bucket[1]
        smallest_size = smallest_bucket[2]
        
        self.get_logger().info(f'最小桶位置: x={smallest_x:.2f}, y={smallest_y:.2f}, size={smallest_size:.2f}')
        
        # 发布最小桶的位置
        self.publish_target_position(smallest_x, smallest_y)
        
    def publish_target_position(self, x, y):
        """发布目标位置到/target_position话题"""
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0  # 默认z为0，因为我们只关心平面位置
        
        self.target_position_publisher.publish(msg)
        self.get_logger().info(f'已发布目标位置: x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = BucketFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()