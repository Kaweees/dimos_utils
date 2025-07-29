#!/usr/bin/env python3
"""
Test LCM publisher for verifying ROS-LCM bridge functionality.
"""

import lcm
import time
import argparse
from lcm_msgs.sensor_msgs import JointState
from lcm_msgs.geometry_msgs import Twist, Pose, Quaternion, Vector3
from lcm_msgs.std_msgs import String, Float64, Int32, Header
import math


def publish_joint_states(lc, channel="/lcm_joint_states"):
    """Publish test joint states."""
    msg = JointState()
    msg.header = Header()
    msg.header.stamp.sec = int(time.time())
    msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
    msg.header.frame_id = "base_link"
    
    # Joint names and values
    msg.name = ["joint1", "joint2", "joint3", "joint4"]
    msg.position = [
        math.sin(time.time()),
        math.cos(time.time()),
        math.sin(time.time() * 0.5),
        math.cos(time.time() * 0.5)
    ]
    msg.velocity = [0.1, -0.1, 0.2, -0.2]
    msg.effort = []
    
    # Set length fields
    msg.name_length = len(msg.name)
    msg.position_length = len(msg.position)
    msg.velocity_length = len(msg.velocity)
    msg.effort_length = 0
    
    lc.publish(f"{channel}#sensor_msgs.JointState", msg.encode())
    print(f"Published JointState to {channel}")


def publish_twist(lc, channel="/lcm_cmd_vel"):
    """Publish test twist command."""
    msg = Twist()
    msg.linear = Vector3()
    msg.angular = Vector3()
    
    # Set some test values
    msg.linear.x = 0.5 * math.sin(time.time() * 0.3)
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.2 * math.cos(time.time() * 0.3)
    
    lc.publish(f"{channel}#geometry_msgs.Twist", msg.encode())
    print(f"Published Twist to {channel}")


def publish_string(lc, channel="/lcm_test_string"):
    """Publish test string message."""
    msg = String()
    msg.data = f"LCM test message at {time.time():.2f}"
    
    lc.publish(f"{channel}#std_msgs.String", msg.encode())
    print(f"Published String to {channel}")


def publish_float(lc, channel="/lcm_test_float"):
    """Publish test float message."""
    msg = Float64()
    msg.data = math.sin(time.time()) * 100.0
    
    lc.publish(f"{channel}#std_msgs.Float64", msg.encode())
    print(f"Published Float64 to {channel}")


def publish_int(lc, channel="/lcm_test_int"):
    """Publish test integer message."""
    msg = Int32()
    msg.data = int(time.time()) % 1000
    
    lc.publish(f"{channel}#std_msgs.Int32", msg.encode())
    print(f"Published Int32 to {channel}")


def main():
    parser = argparse.ArgumentParser(description='Test LCM publisher for ROS-LCM bridge')
    parser.add_argument('--rate', type=float, default=1.0, help='Publishing rate in Hz')
    args = parser.parse_args()
    
    lc = lcm.LCM()
    
    print(f"Starting LCM test publisher at {args.rate} Hz")
    print("Publishing to channels:")
    print("  - /lcm_joint_states#sensor_msgs.JointState")
    print("  - /lcm_cmd_vel#geometry_msgs.Twist")
    print("  - /lcm_test_string#std_msgs.String")
    print("  - /lcm_test_float#std_msgs.Float64")
    print("  - /lcm_test_int#std_msgs.Int32")
    
    try:
        while True:
            publish_joint_states(lc)
            publish_twist(lc)
            publish_string(lc)
            publish_float(lc)
            publish_int(lc)
            
            time.sleep(1.0 / args.rate)
            
    except KeyboardInterrupt:
        print("\nShutting down LCM test publisher")


if __name__ == '__main__':
    main()