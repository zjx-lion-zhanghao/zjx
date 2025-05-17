#!/usr/bin/env python3
"""
测试相机节点 - 产生随机的桶位置和大小数据
用于测试深度相机->桶过滤->控制系统的数据流程
支持多种测试模式：完全随机、固定位置随机大小、固定数量等
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import random
import time
import argparse
import sys

class RandomBucketCameraNode(Node):
    def __init__(self, test_mode="random", fixed_count=None, fixed_position=False):
        super().__init__('random_bucket_camera_node')
        
        # 创建发布者，发布桶列表数据
        self.bucket_list_publisher = self.create_publisher(
            Float32MultiArray, '/bucket_list', 10)
        
        # 创建一个状态发布者，用于监控和测试目的
        self.status_publisher = self.create_publisher(
            String, '/camera_status', 10)
        
        # 参数设置
        self.test_mode = test_mode      # 测试模式: "random", "fixed_position", "fixed_count"
        self.fixed_count = fixed_count  # 固定桶数量模式下的数量
        self.fixed_position = fixed_position  # 是否使用固定位置
        
        # 根据需求，修改桶的数量上限为3个
        self.min_buckets = 2 if fixed_count is None else fixed_count  # 最少桶数量
        self.max_buckets = 3 if fixed_count is None else fixed_count  # 最多桶数量 (限制为3个桶)
        self.publish_rate = 0.5    # 发布频率 (Hz)
        self.position_range = 1.0  # 位置范围限制在更小的区域 (-1.0 ~ 1.0 米)
        self.min_bucket_size = 0.1 # 最小桶尺寸
        self.max_bucket_size = 0.8 # 最大桶尺寸
        
        # 固定位置模式下的预定义位置 (只保留3个位置)
        self.fixed_positions = [
            (-0.7, -0.5),   # 左前方
            (0.2, 0.6),     # 右前方
            (-0.3, 0.1)     # 中间
        ]
        
        # 记录最小桶的信息，用于验证
        self.smallest_bucket = None
        self.publish_count = 0  # 记录发布次数
        
        # 创建定时器，定期发布模拟数据
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_random_buckets)
        
        msg = String()
        msg.data = f"相机节点启动: 测试模式={self.test_mode}, 桶数量={self.min_buckets}-{self.max_buckets}"
        self.status_publisher.publish(msg)
        self.get_logger().info(f'随机桶测试相机节点已启动 [模式: {self.test_mode}]')
        
    def publish_random_buckets(self):
        """发布随机的桶位置和大小数据"""
        self.publish_count += 1
        
        # 根据测试模式决定桶的数量，限制最多3个桶
        if self.test_mode == "fixed_count" or self.fixed_count is not None:
            bucket_count = min(self.fixed_count, 3)  # 确保不超过3个桶
        else:
            bucket_count = random.randint(self.min_buckets, self.max_buckets)  # max_buckets已经设为3
        
        # 随机生成桶的数据
        buckets = []
        min_size_index = random.randint(0, bucket_count-1)  # 随机选择一个桶作为最小桶
        
        for i in range(bucket_count):
            # 确定位置 - 修改坐标生成逻辑
            if self.fixed_position and i < len(self.fixed_positions):
                x, y = self.fixed_positions[i]
            else:
                # 随机位置，确保生成的坐标在合理范围内
                if i == 0:  # 第一个桶位置在左侧区域
                    x = random.uniform(-self.position_range, -0.2)
                    y = random.uniform(-0.7, 0.7)
                elif i == 1:  # 第二个桶位置在右侧区域
                    x = random.uniform(0.2, self.position_range)
                    y = random.uniform(-0.7, 0.7)
                else:  # 第三个桶位置在中间区域
                    x = random.uniform(-0.3, 0.3)
                    y = random.uniform(-0.5, 0.5)
            
            # 随机大小，确保有一个桶是明显最小的
            if i == min_size_index:
                # 最小的桶尺寸更小，确保能被明确区分
                size = random.uniform(self.min_bucket_size, self.min_bucket_size + 0.15)
                self.smallest_bucket = (x, y, size)
                self.get_logger().info(f"最小的桶 (索引 {i}): x={x:.2f}, y={y:.2f}, size={size:.2f}")
            else:
                # 其他桶的大小必须明显大于最小桶
                size = random.uniform(self.min_bucket_size + 0.35, self.max_bucket_size)
            
            buckets.append([x, y, size])
        
        # 打印所有桶的信息
        self.get_logger().info(f"生成了 {bucket_count} 个随机桶:")
        for i, bucket in enumerate(buckets):
            self.get_logger().info(f"  桶 {i+1}: x={bucket[0]:.2f}, y={bucket[1]:.2f}, size={bucket[2]:.2f}")
        
        # 验证是否找到了最小的桶
        if self.smallest_bucket is None:
            self.get_logger().error("错误：未能确定最小的桶！")
            return
            
        # 转换为一维列表
        data = []
        for bucket in buckets:
            data.extend(bucket)
            
        # 创建消息
        msg = Float32MultiArray()
        msg.data = data
        
        # 发布消息
        self.bucket_list_publisher.publish(msg)
        
        # 发布状态信息，用于测试和监控
        status_msg = String()
        status_msg.data = f"CAMERA_DATA|{self.publish_count}|{bucket_count}|{self.smallest_bucket[0]:.2f}|{self.smallest_bucket[1]:.2f}|{self.smallest_bucket[2]:.2f}"
        self.status_publisher.publish(status_msg)
        
        # 增强日志输出，更清晰地显示最小桶的位置用于调试
        self.get_logger().info(f"已发布随机桶数据 (第 {self.publish_count} 次发布)")
        self.get_logger().info(f"【关键信息】最小桶位置: x={self.smallest_bucket[0]:.3f}, y={self.smallest_bucket[1]:.3f}, 尺寸={self.smallest_bucket[2]:.3f}")

def main(args=None):
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='随机桶测试相机节点')
    parser.add_argument('--mode', type=str, default='random',
                        choices=['random', 'fixed_position', 'fixed_count'],
                        help='测试模式: random (完全随机), fixed_position (固定位置), fixed_count (固定数量)')
    parser.add_argument('--count', type=int, 
                        help='桶的数量 (仅用于 fixed_count 模式, 最大值为3)', 
                        choices=[1, 2, 3], default=3)
    parser.add_argument('--fixed-pos', action='store_true', 
                        help='使用固定的预定义位置')
    parser.add_argument('--rate', type=float, default=0.5,
                        help='发布频率 (Hz)')
    
    # 解析命令行参数，但保留未知参数给rclpy处理
    parsed_args, remaining_args = parser.parse_known_args()
    
    # 打印启动信息
    print(f"启动随机桶相机节点: 模式={parsed_args.mode}, 桶数量={parsed_args.count}, 固定位置={parsed_args.fixed_pos}")
    
    rclpy.init(args=remaining_args)
    
    # 创建节点
    node = RandomBucketCameraNode(
        test_mode=parsed_args.mode,
        fixed_count=parsed_args.count,
        fixed_position=parsed_args.fixed_pos
    )
    
    # 如果用户指定了发布频率，则更新
    if parsed_args.rate != 0.5:
        node.publish_rate = parsed_args.rate
        node.timer.timer_period_ns = int(1_000_000_000 / parsed_args.rate)
        node.get_logger().info(f"已设置发布频率为: {parsed_args.rate} Hz")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断测试')
    finally:
        node.get_logger().info(f'测试完成: 总共发布了 {node.publish_count} 次数据')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
