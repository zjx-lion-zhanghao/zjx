#!/usr/bin/env python3
"""
测试脚本 - 测试深度相机数据流传输和处理
此脚本用于测试从深度相机到控制系统的完整数据流程
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import time

class TestDeepCameraFlow(Node):
    """测试深度相机数据流程的节点"""
    
    def __init__(self):
        super().__init__('test_deep_camera_flow')
        
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
        
        # 创建定时器发布模拟数据
        self.timer = self.create_timer(1.0, self.publish_test_data)
        self.test_timer = self.create_timer(5.0, self.evaluate_test)
        
        self.get_logger().info('深度相机数据流测试节点已启动')
        
    def publish_test_data(self):
        """发布测试用的桶数据"""
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
        self.get_logger().info('已发布测试桶列表数据')
        
    def target_position_callback(self, msg):
        """验证接收到的目标位置是否正确"""
        self.target_position = msg
        self.received_target = True
        
        # 预期的目标位置应该是最小桶的位置
        expected_x = -0.2
        expected_y = 0.1
        
        # 检查与预期值的差异（允许小误差）
        x_diff = abs(msg.x - expected_x)
        y_diff = abs(msg.y - expected_y)
        
        if x_diff < 0.05 and y_diff < 0.05:
            self.test_passed = True
            self.get_logger().info('✅ 测试通过！接收到的目标位置与预期最小桶位置相符')
        else:
            self.get_logger().warning(f'❌ 测试失败！接收到的位置 [{msg.x:.2f}, {msg.y:.2f}] 与预期 [{expected_x}, {expected_y}] 不符')
            
        self.get_logger().info(f'接收到目标位置: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')
    
    def evaluate_test(self):
        """评估测试结果"""
        if not self.received_target:
            self.get_logger().error('❌ 测试失败！未收到目标位置消息，请检查 bucket_filter_node 是否正常工作')
        elif self.test_passed:
            self.get_logger().info('✅ 测试成功完成！数据流正常工作')
        else:
            self.get_logger().warning('❓ 测试完成，但结果不符合预期')
            
        # 输出测试总结
        self.get_logger().info('=== 测试总结 ===')
        self.get_logger().info(f'桶列表发布: ✅')
        self.get_logger().info(f'目标位置接收: {"✅" if self.received_target else "❌"}')
        self.get_logger().info(f'目标位置正确性: {"✅" if self.test_passed else "❌"}')
        
        # 如果已接收到目标位置，检查 control_v1 是否正确处理
        # 这需要另一种验证方法，因为我们无法直接读取 control_v1 内部状态
        if self.received_target:
            self.get_logger().info('提示: 请检查控制节点日志，确认其是否收到并正确处理了目标位置')

def main(args=None):
    rclpy.init(args=args)
    node = TestDeepCameraFlow()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断测试')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
