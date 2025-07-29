#!/usr/bin/env python3
"""
Test ROS2 publisher for verifying ROS-LCM bridge functionality.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Int32, Header
import math
import time


class TestRosPublisher(Node):
    def __init__(self):
        super().__init__('test_ros_publisher')
        
        # Create publishers
        self.joint_pub = self.create_publisher(JointState, 'ros_joint_states', 10)
        self.twist_pub = self.create_publisher(Twist, 'ros_cmd_vel', 10)
        self.string_pub = self.create_publisher(String, 'ros_test_string', 10)
        self.float_pub = self.create_publisher(Float64, 'ros_test_float', 10)
        self.int_pub = self.create_publisher(Int32, 'ros_test_int', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0, self.publish_all)
        
        self.get_logger().info('ROS2 test publisher started')
        self.get_logger().info('Publishing to topics:')
        self.get_logger().info('  - ros_joint_states (sensor_msgs/msg/JointState)')
        self.get_logger().info('  - ros_cmd_vel (geometry_msgs/msg/Twist)')
        self.get_logger().info('  - ros_test_string (std_msgs/msg/String)')
        self.get_logger().info('  - ros_test_float (std_msgs/msg/Float64)')
        self.get_logger().info('  - ros_test_int (std_msgs/msg/Int32)')
    
    def publish_all(self):
        """Publish all test messages."""
        current_time = time.time()
        
        # Publish JointState
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = "base_link"
        joint_msg.name = ["joint_a", "joint_b", "joint_c"]
        joint_msg.position = [
            math.cos(current_time),
            math.sin(current_time),
            math.cos(current_time * 0.7)
        ]
        joint_msg.velocity = [0.5, -0.5, 0.3]
        joint_msg.effort = []
        self.joint_pub.publish(joint_msg)
        self.get_logger().info('Published JointState')
        
        # Publish Twist
        twist_msg = Twist()
        twist_msg.linear.x = 0.3 * math.cos(current_time * 0.5)
        twist_msg.linear.y = 0.1
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.15 * math.sin(current_time * 0.5)
        self.twist_pub.publish(twist_msg)
        self.get_logger().info('Published Twist')
        
        # Publish String
        string_msg = String()
        string_msg.data = f"ROS2 test message at {current_time:.2f}"
        self.string_pub.publish(string_msg)
        self.get_logger().info('Published String')
        
        # Publish Float64
        float_msg = Float64()
        float_msg.data = math.cos(current_time) * 50.0
        self.float_pub.publish(float_msg)
        self.get_logger().info('Published Float64')
        
        # Publish Int32
        int_msg = Int32()
        int_msg.data = int(current_time * 10) % 500
        self.int_pub.publish(int_msg)
        self.get_logger().info('Published Int32')


def main():
    rclpy.init()
    
    node = TestRosPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()