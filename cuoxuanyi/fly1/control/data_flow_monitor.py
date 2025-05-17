#!/usr/bin/env python3
"""
数据流监控节点 - 用于监控和验证随机桶数据从相机到控制节点的传输流程
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point
import time

class DataFlowMonitor(Node):
    def __init__(self):
        super().__init__('data_flow_monitor')
        
        # 记录接收到的消息情况
        self.camera_status = None
        self.bucket_list_data = None
        self.target_position = None
        self.last_camera_time = None
        self.last_bucket_filter_time = None
        self.last_target_time = None
        
        # 数据统计
        self.camera_count = 0
        self.bucket_filter_count = 0
        self.target_position_count = 0
        
        # 创建订阅者，监听整个数据流
        self.camera_status_sub = self.create_subscription(
            String, '/camera_status', self.camera_status_callback, 10)
            
        self.bucket_list_sub = self.create_subscription(
            Float32MultiArray, '/bucket_list', self.bucket_list_callback, 10)
            
        self.target_position_sub = self.create_subscription(
            Point, '/target_position', self.target_position_callback, 10)
        
        # 创建一个定时器，定期检查并输出状态
        self.timer = self.create_timer(2.0, self.report_status)
        
        self.get_logger().info('数据流监控节点已启动，监控各节点间的数据传输')
        
    def camera_status_callback(self, msg):
        """处理相机状态信息"""
        self.camera_status = msg.data
        self.last_camera_time = self.get_clock().now()
        self.camera_count += 1
        
        # 解析相机发送的数据
        try:
            parts = msg.data.split('|')
            if parts[0] == 'CAMERA_DATA':
                seq = parts[1]
                bucket_count = parts[2]
                smallest_x = float(parts[3])
                smallest_y = float(parts[4])
                smallest_size = float(parts[5])
                self.get_logger().info(f"相机数据 #{seq}: {bucket_count}个桶, 最小桶: x={smallest_x:.2f}, y={smallest_y:.2f}, size={smallest_size:.2f}")
        except Exception as e:
            self.get_logger().error(f"解析相机状态数据错误: {e}")
    
    def bucket_list_callback(self, msg):
        """处理接收到的桶列表数据"""
        self.bucket_list_data = msg.data
        self.last_bucket_filter_time = self.get_clock().now()
        self.bucket_filter_count += 1
        
        # 解析桶数据
        bucket_count = len(msg.data) // 3
        self.get_logger().info(f"接收到桶列表数据: {bucket_count}个桶")
        
    def target_position_callback(self, msg):
        """处理目标位置数据"""
        self.target_position = msg
        self.last_target_time = self.get_clock().now()
        self.target_position_count += 1
        
        self.get_logger().info(f"接收到目标位置: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
        
        # 检查延迟
        if self.last_bucket_filter_time:
            delay = (self.last_target_time.nanoseconds - self.last_bucket_filter_time.nanoseconds) / 1e9
            self.get_logger().info(f"桶过滤->目标位置 延迟: {delay:.3f}秒")
    
    def report_status(self):
        """定期报告整个数据流的状态"""
        now = self.get_clock().now()
        
        self.get_logger().info("=== 数据流状态报告 ===")
        self.get_logger().info(f"相机数据: {self.camera_count} 条消息")
        self.get_logger().info(f"桶列表数据: {self.bucket_filter_count} 条消息")
        self.get_logger().info(f"目标位置数据: {self.target_position_count} 条消息")
        
        # 检查各节点是否活跃
        camera_active = True
        bucket_filter_active = True
        target_position_active = True
        
        if self.last_camera_time:
            camera_time_diff = (now.nanoseconds - self.last_camera_time.nanoseconds) / 1e9
            camera_active = camera_time_diff < 5.0  # 5秒内有数据则认为活跃
            self.get_logger().info(f"相机节点状态: {'活跃' if camera_active else '不活跃'} ({camera_time_diff:.1f}秒前)")
        else:
            self.get_logger().info("相机节点状态: 未接收数据")
            
        if self.last_bucket_filter_time:
            filter_time_diff = (now.nanoseconds - self.last_bucket_filter_time.nanoseconds) / 1e9
            bucket_filter_active = filter_time_diff < 5.0
            self.get_logger().info(f"桶过滤节点状态: {'活跃' if bucket_filter_active else '不活跃'} ({filter_time_diff:.1f}秒前)")
        else:
            self.get_logger().info("桶过滤节点状态: 未接收数据")
            
        if self.last_target_time:
            target_time_diff = (now.nanoseconds - self.last_target_time.nanoseconds) / 1e9
            target_position_active = target_time_diff < 5.0
            self.get_logger().info(f"目标位置状态: {'活跃' if target_position_active else '不活跃'} ({target_time_diff:.1f}秒前)")
        else:
            self.get_logger().info("目标位置状态: 未接收数据")
            
        # 检查整体数据流是否正常
        if camera_active and bucket_filter_active and target_position_active:
            self.get_logger().info("数据流状态: 正常")
        else:
            self.get_logger().warn("数据流状态: 异常")
            
            # 诊断问题
            if not camera_active:
                self.get_logger().warn(" - 相机节点未发送数据")
            if camera_active and not bucket_filter_active:
                self.get_logger().warn(" - 桶过滤节点未正确处理相机数据")
            if bucket_filter_active and not target_position_active:
                self.get_logger().warn(" - 控制节点未接收目标位置数据")
        
        self.get_logger().info("========================")


def main(args=None):
    rclpy.init(args=args)
    node = DataFlowMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
