#!/usr/bin/env python3
"""
Bidirectional ROS2-LCM bridge with automatic message type mapping and feedback prevention.
"""

import rclpy
from rclpy.node import Node
import lcm
import threading
import importlib
import time
import hashlib
import json
import sys
import traceback
from typing import Dict, Any, Set, Tuple, Optional
import argparse
import signal

# Import LCM message types
from lcm_msgs import std_msgs as lcm_std_msgs
from lcm_msgs import sensor_msgs as lcm_sensor_msgs
from lcm_msgs import geometry_msgs as lcm_geometry_msgs
from lcm_msgs import nav_msgs as lcm_nav_msgs

# Import ROS2 message types
from std_msgs import msg as ros_std_msgs
from sensor_msgs import msg as ros_sensor_msgs
from geometry_msgs import msg as ros_geometry_msgs
from nav_msgs import msg as ros_nav_msgs


class MessageConverter:
    """Handles conversion between ROS2 and LCM messages."""
    
    @staticmethod
    def get_message_hash(msg_data: bytes) -> str:
        """Create a hash of message content for comparison."""
        return hashlib.md5(msg_data).hexdigest()
    
    @staticmethod
    def ros_to_lcm_type_name(ros_type: str) -> str:
        """Convert ROS type name to LCM type name."""
        # Example: sensor_msgs/msg/JointState -> sensor_msgs.JointState
        parts = ros_type.split('/')
        if len(parts) == 3 and parts[1] == 'msg':
            return f"{parts[0]}.{parts[2]}"
        return ros_type.replace('/', '.')
    
    @staticmethod
    def lcm_to_ros_type_name(lcm_type: str) -> str:
        """Convert LCM type name to ROS type name."""
        # Example: sensor_msgs.JointState -> sensor_msgs/msg/JointState
        parts = lcm_type.split('.')
        if len(parts) == 2:
            return f"{parts[0]}/msg/{parts[1]}"
        return lcm_type.replace('.', '/')
    
    @staticmethod
    def is_message_type_supported(ros_type: str = None, lcm_type: str = None) -> bool:
        """Check if a message type is supported by trying to import it."""
        if ros_type:
            try:
                parts = ros_type.split('/')
                if len(parts) == 3 and parts[1] == 'msg':
                    module = importlib.import_module(f"{parts[0]}.{parts[1]}")
                    getattr(module, parts[2])
                    return True
            except:
                return False
                
        if lcm_type:
            try:
                module_name, class_name = lcm_type.rsplit('.', 1)
                module = importlib.import_module(f"lcm_msgs.{module_name}")
                getattr(module, class_name)
                return True
            except:
                return False
                
        return False
    
    @staticmethod
    def get_lcm_message_class(lcm_type: str):
        """Get LCM message class from type string."""
        try:
            module_name, class_name = lcm_type.rsplit('.', 1)
            module = importlib.import_module(f"lcm_msgs.{module_name}")
            return getattr(module, class_name)
        except Exception as e:
            print(f"Failed to import LCM message type {lcm_type}: {e}")
            return None
    
    @staticmethod
    def get_ros_message_class(ros_type: str):
        """Get ROS message class from type string."""
        try:
            parts = ros_type.split('/')
            if len(parts) == 3 and parts[1] == 'msg':
                module = importlib.import_module(f"{parts[0]}.{parts[1]}")
                return getattr(module, parts[2])
            return None
        except Exception as e:
            print(f"Failed to import ROS message type {ros_type}: {e}")
            return None
    
    @staticmethod
    def convert_ros_to_lcm(ros_msg, lcm_msg_class):
        """Convert a ROS message to LCM message."""
        lcm_msg = lcm_msg_class()
        
        # Copy common fields
        for field_name in dir(ros_msg):
            if field_name.startswith('_'):
                continue
            
            try:
                ros_value = getattr(ros_msg, field_name)
                
                # Handle array length fields for LCM
                if hasattr(lcm_msg, f"{field_name}_length") and isinstance(ros_value, (list, tuple)):
                    setattr(lcm_msg, f"{field_name}_length", len(ros_value))
                
                # Special handling for nested messages
                if hasattr(lcm_msg, field_name):
                    lcm_field_type = type(getattr(lcm_msg, field_name))
                    
                    # If it's a nested message, recursively convert
                    if hasattr(lcm_field_type, '__module__') and 'lcm_msgs' in lcm_field_type.__module__:
                        # Find corresponding ROS type
                        ros_field_type = type(ros_value)
                        if hasattr(ros_field_type, '__module__'):
                            lcm_value = MessageConverter.convert_ros_to_lcm(ros_value, lcm_field_type)
                            setattr(lcm_msg, field_name, lcm_value)
                        else:
                            setattr(lcm_msg, field_name, ros_value)
                    else:
                        setattr(lcm_msg, field_name, ros_value)
                
            except Exception as e:
                # Skip fields that don't exist or can't be converted
                pass
        
        return lcm_msg
    
    @staticmethod
    def convert_lcm_to_ros(lcm_msg, ros_msg_class):
        """Convert an LCM message to ROS message."""
        ros_msg = ros_msg_class()
        
        # Copy common fields
        for field_name in dir(lcm_msg):
            if field_name.startswith('_') or field_name.endswith('_length'):
                continue
            
            try:
                lcm_value = getattr(lcm_msg, field_name)
                
                # Special handling for nested messages
                if hasattr(ros_msg, field_name):
                    ros_field_type = type(getattr(ros_msg, field_name))
                    
                    # If it's a nested message, recursively convert
                    if hasattr(ros_field_type, '__module__') and ros_field_type.__module__.endswith('.msg'):
                        # Find corresponding LCM type
                        lcm_field_type = type(lcm_value)
                        if hasattr(lcm_field_type, '__module__'):
                            ros_value = MessageConverter.convert_lcm_to_ros(lcm_value, ros_field_type)
                            setattr(ros_msg, field_name, ros_value)
                        else:
                            setattr(ros_msg, field_name, lcm_value)
                    else:
                        setattr(ros_msg, field_name, lcm_value)
                
            except Exception as e:
                # Skip fields that don't exist or can't be converted
                pass
        
        return ros_msg


class TopicState:
    """Tracks state for a single topic to prevent feedback loops."""
    
    def __init__(self, topic_name: str):
        self.topic_name = topic_name
        self.last_ros_to_lcm_hash = None
        self.last_lcm_to_ros_hash = None
        self.last_ros_to_lcm_time = 0
        self.last_lcm_to_ros_time = 0
        self.message_count_ros_to_lcm = 0
        self.message_count_lcm_to_ros = 0
    
    def should_forward_ros_to_lcm(self, msg_hash: str) -> bool:
        """Check if ROS message should be forwarded to LCM."""
        # Don't forward if this is the same message we just sent from LCM to ROS
        if msg_hash == self.last_lcm_to_ros_hash:
            return False
        
        # Don't forward if we just sent this exact message
        if msg_hash == self.last_ros_to_lcm_hash:
            time_diff = time.time() - self.last_ros_to_lcm_time
            # Allow resending after 1 second
            if time_diff < 1.0:
                return False
        
        return True
    
    def should_forward_lcm_to_ros(self, msg_hash: str) -> bool:
        """Check if LCM message should be forwarded to ROS."""
        # Don't forward if this is the same message we just sent from ROS to LCM
        if msg_hash == self.last_ros_to_lcm_hash:
            return False
        
        # Don't forward if we just sent this exact message
        if msg_hash == self.last_lcm_to_ros_hash:
            time_diff = time.time() - self.last_lcm_to_ros_time
            # Allow resending after 1 second
            if time_diff < 1.0:
                return False
        
        return True
    
    def mark_ros_to_lcm(self, msg_hash: str):
        """Mark that we sent a message from ROS to LCM."""
        self.last_ros_to_lcm_hash = msg_hash
        self.last_ros_to_lcm_time = time.time()
        self.message_count_ros_to_lcm += 1
    
    def mark_lcm_to_ros(self, msg_hash: str):
        """Mark that we sent a message from LCM to ROS."""
        self.last_lcm_to_ros_hash = msg_hash
        self.last_lcm_to_ros_time = time.time()
        self.message_count_lcm_to_ros += 1


class RosLcmBridge(Node):
    """Bidirectional ROS2-LCM bridge."""
    
    def __init__(self):
        super().__init__('ros_lcm_bridge')
        
        # Initialize LCM
        self.lcm = lcm.LCM()
        
        # Topic state tracking
        self.topic_states: Dict[str, TopicState] = {}
        
        # Active subscriptions and publishers
        self.ros_subscribers = {}
        self.ros_publishers = {}
        self.lcm_subscriptions = {}
        
        # Track discovered LCM channels
        self.discovered_lcm_channels = set()
        
        # Message type mapping - no longer needed, we'll do it automatically!
        
        # Subscribe to all LCM channels for discovery
        self.lcm.subscribe(".*", self.lcm_discovery_handler)
        
        # Start periodic discovery
        self.create_timer(2.0, self.discover_topics)
        
        # Start LCM handler thread
        self.lcm_thread = threading.Thread(target=self.lcm_handler, daemon=True)
        self.lcm_thread.start()
        
        self.get_logger().info("ROS-LCM Bridge started")
    
    def get_or_create_topic_state(self, topic_name: str) -> TopicState:
        """Get or create topic state."""
        if topic_name not in self.topic_states:
            self.topic_states[topic_name] = TopicState(topic_name)
        return self.topic_states[topic_name]
    
    def lcm_discovery_handler(self, channel, data):
        """Handle LCM messages for channel discovery and forwarding."""
        try:
            # Extract channel name and type if present
            if '#' in channel:
                topic_name, lcm_type = channel.rsplit('#', 1)
            else:
                # Try to determine type from data (this is harder without type info)
                return
            
            # Check if this is a new channel we haven't seen
            if channel not in self.discovered_lcm_channels:
                self.discovered_lcm_channels.add(channel)
                
                # Check if we should create a bridge for this channel
                # Important: Only create bridge if:
                # 1. We know how to convert this message type
                # 2. This is NOT a channel we're already publishing to from ROS
                # 3. We don't already have a ROS topic with this name
                
                # The key insight: if topic_name starts with '/', it might be from ROS
                # But we need to check if this specific CHANNEL is one we're publishing to
                
                should_create_bridge = True
                
                # Check if this exact channel is one we publish to from ROS
                for ros_topic in self.ros_subscribers:
                    expected_channel = f"{ros_topic}#{lcm_type}"
                    if channel == expected_channel:
                        # This channel is created by our ROS->LCM bridge
                        should_create_bridge = False
                        break
                
                if should_create_bridge:
                    # Automatically convert LCM type to ROS type
                    ros_type = MessageConverter.lcm_to_ros_type_name(lcm_type)
                    
                    # Check if both types are supported
                    if MessageConverter.is_message_type_supported(ros_type=ros_type, lcm_type=lcm_type):
                        self.get_logger().info(f"Discovered new LCM channel: {channel}")
                        self.create_lcm_to_ros_bridge(topic_name, lcm_type, ros_type)
                    else:
                        self.get_logger().debug(f"Unsupported message type for channel {channel}: {lcm_type}")
            
            # Forward the message to the appropriate handler
            if channel in self.lcm_subscriptions:
                callback = self.lcm_subscriptions[channel]
                if callback:
                    callback(channel, data)
                    
        except Exception as e:
            # Silently ignore errors in discovery to avoid spam
            pass
    
    def discover_topics(self):
        """Discover new topics on both ROS and LCM."""
        # Discover ROS topics
        try:
            topic_list = self.get_topic_names_and_types()
            for topic_name, topic_types in topic_list:
                # Skip if we already have a subscriber for this topic
                if topic_name in self.ros_subscribers:
                    continue
                    
                # Skip if this topic exists because we created it from LCM
                if topic_name in self.ros_publishers:
                    continue
                    
                # Skip system topics
                if topic_name.startswith('/rosout') or topic_name.startswith('/parameter'):
                    continue
                    
                for topic_type in topic_types:
                    # Automatically convert ROS type to LCM type
                    lcm_type = MessageConverter.ros_to_lcm_type_name(topic_type)
                    
                    # Check if both types are supported
                    if MessageConverter.is_message_type_supported(ros_type=topic_type, lcm_type=lcm_type):
                        self.create_ros_to_lcm_bridge(topic_name, topic_type)
                        break
        except Exception as e:
            self.get_logger().error(f"Error discovering ROS topics: {e}")
    
    def create_ros_to_lcm_bridge(self, ros_topic: str, ros_type: str):
        """Create a bridge from ROS topic to LCM channel."""
        try:
            ros_msg_class = MessageConverter.get_ros_message_class(ros_type)
            lcm_type = MessageConverter.ros_to_lcm_type_name(ros_type)
            lcm_msg_class = MessageConverter.get_lcm_message_class(lcm_type)
            
            if not ros_msg_class or not lcm_msg_class:
                return
            
            # Check if already created
            if ros_topic in self.ros_subscribers:
                return
            
            # Create ROS subscriber
            def ros_callback(ros_msg):
                try:
                    # Convert message
                    lcm_msg = MessageConverter.convert_ros_to_lcm(ros_msg, lcm_msg_class)
                    
                    # Check for feedback prevention
                    lcm_data = lcm_msg.encode()
                    msg_hash = MessageConverter.get_message_hash(lcm_data)
                    
                    topic_state = self.get_or_create_topic_state(ros_topic)
                    if topic_state.should_forward_ros_to_lcm(msg_hash):
                        # Publish to LCM with type annotation
                        lcm_channel = f"{ros_topic}#{lcm_type}"
                        self.lcm.publish(lcm_channel, lcm_data)
                        topic_state.mark_ros_to_lcm(msg_hash)
                        
                except Exception as e:
                    self.get_logger().error(f"Error in ROS->LCM bridge for {ros_topic}: {e}")
                    traceback.print_exc()
            
            self.ros_subscribers[ros_topic] = self.create_subscription(
                ros_msg_class,
                ros_topic,
                ros_callback,
                10
            )
            
            self.get_logger().info(f"Created ROS->LCM bridge: {ros_topic} ({ros_type}) -> {lcm_type}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to create ROS->LCM bridge for {ros_topic}: {e}")
            traceback.print_exc()
    
    def create_lcm_to_ros_bridge(self, ros_topic: str, lcm_type: str, ros_type: str):
        """Create a bridge from LCM channel to ROS topic."""
        try:
            lcm_msg_class = MessageConverter.get_lcm_message_class(lcm_type)
            ros_msg_class = MessageConverter.get_ros_message_class(ros_type)
            
            if not lcm_msg_class or not ros_msg_class:
                return
            
            # Check if already created
            lcm_channel = f"{ros_topic}#{lcm_type}"
            if lcm_channel in self.lcm_subscriptions:
                return
            
            # Create ROS publisher if not exists
            if ros_topic not in self.ros_publishers:
                self.ros_publishers[ros_topic] = self.create_publisher(
                    ros_msg_class,
                    ros_topic,
                    10
                )
            
            def lcm_callback(channel, data):
                try:
                    # Only process if this is the specific channel we're interested in
                    if channel != lcm_channel and channel != ros_topic:
                        return
                    
                    # Decode LCM message
                    lcm_msg = lcm_msg_class.decode(data)
                    
                    # Check for feedback prevention
                    msg_hash = MessageConverter.get_message_hash(data)
                    topic_state = self.get_or_create_topic_state(ros_topic)
                    
                    if topic_state.should_forward_lcm_to_ros(msg_hash):
                        # Convert to ROS
                        ros_msg = MessageConverter.convert_lcm_to_ros(lcm_msg, ros_msg_class)
                        
                        # Publish to ROS
                        self.ros_publishers[ros_topic].publish(ros_msg)
                        topic_state.mark_lcm_to_ros(msg_hash)
                        
                except Exception as e:
                    self.get_logger().error(f"Error in LCM->ROS bridge for {channel}: {e}")
                    traceback.print_exc()
            
            # Store the callback for this specific channel
            self.lcm_subscriptions[lcm_channel] = lcm_callback
            
            # Use the discovery handler to process messages for this channel
            # The discovery handler already subscribes to all channels
            
            self.get_logger().info(f"Created LCM->ROS bridge: {lcm_type} -> {ros_topic} ({ros_type})")
            
        except Exception as e:
            self.get_logger().error(f"Failed to create LCM->ROS bridge for {ros_topic}: {e}")
            traceback.print_exc()
    
    def lcm_handler(self):
        """Handle LCM messages in separate thread."""
        while rclpy.ok():
            try:
                self.lcm.handle_timeout(100)  # 100ms timeout
            except Exception as e:
                print(f"LCM handler error: {e}")
                time.sleep(0.1)
    
    def print_statistics(self):
        """Print bridge statistics."""
        print("\n=== ROS-LCM Bridge Statistics ===")
        for topic_name, state in self.topic_states.items():
            print(f"\nTopic: {topic_name}")
            print(f"  ROS->LCM messages: {state.message_count_ros_to_lcm}")
            print(f"  LCM->ROS messages: {state.message_count_lcm_to_ros}")


def main():
    parser = argparse.ArgumentParser(description='Bidirectional ROS2-LCM Bridge')
    args = parser.parse_args()
    
    rclpy.init()
    
    bridge = RosLcmBridge()
    
    # Setup signal handler for statistics
    def signal_handler(sig, frame):
        bridge.print_statistics()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(bridge)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        bridge.print_statistics()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()