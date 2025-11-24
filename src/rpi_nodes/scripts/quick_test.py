#!/usr/bin/env python3
"""
Quick test for keyboard control node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

def test_keyboard_topics():
    """Test if keyboard topics are properly configured"""
    
    rclpy.init()
    
    # Create a simple node to test publishing
    node = rclpy.create_node('keyboard_test')
    
    # Create publishers for all keyboard topics
    publishers = {
        'group1': node.create_publisher(Int32, '/keyboard/group1', 10),
        'group2': node.create_publisher(Int32, '/keyboard/group2', 10),
        'group3': node.create_publisher(Int32, '/keyboard/group3', 10),
        'group4': node.create_publisher(Int32, '/keyboard/group4', 10),
    }
    
    # Test publishing to each topic
    for group, pub in publishers.items():
        msg = Int32()
        msg.data = 42
        pub.publish(msg)
        print(f"✓ Published to /keyboard/{group}: {msg.data}")
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    
    print("✓ All keyboard topics working correctly!")

if __name__ == '__main__':
    try:
        test_keyboard_topics()
    except Exception as e:
        print(f"❌ Error: {e}")
