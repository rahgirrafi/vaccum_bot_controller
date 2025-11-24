#!/usr/bin/env python3
"""
Test script for keyboard control node.
This script subscribes to all 4 keyboard topics and displays the received values.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class KeyboardTestNode(Node):
    """Test node to monitor keyboard control topics"""
    
    def __init__(self):
        super().__init__('keyboard_test_node')
        
        # Create subscriptions for all 4 groups
        self.subscriptions = []
        
        self.group1_sub = self.create_subscription(
            Int32, '/keyboard/group1', 
            lambda msg: self.callback(msg, 'Group1 (q,a,z)'), 10)
        
        self.group2_sub = self.create_subscription(
            Int32, '/keyboard/group2', 
            lambda msg: self.callback(msg, 'Group2 (w,s,x)'), 10)
        
        self.group3_sub = self.create_subscription(
            Int32, '/keyboard/group3', 
            lambda msg: self.callback(msg, 'Group3 (r,f,v)'), 10)
        
        self.group4_sub = self.create_subscription(
            Int32, '/keyboard/group4', 
            lambda msg: self.callback(msg, 'Group4 (t,g,b)'), 10)
        
        self.get_logger().info('Keyboard test node started - monitoring all keyboard topics')
        self.get_logger().info('Listening to: /keyboard/group1, /keyboard/group2, /keyboard/group3, /keyboard/group4')
    
    def callback(self, msg, group_name):
        """Callback for keyboard messages"""
        value = msg.data
        direction = ""
        if value == 1:
            direction = "POSITIVE"
        elif value == -1:
            direction = "NEGATIVE"  
        elif value == 0:
            direction = "ZERO"
        
        self.get_logger().info(f'{group_name}: {value} ({direction})')

def main(args=None):
    try:
        rclpy.init(args=args)
        
        node = KeyboardTestNode()
        
        print('Keyboard test node started - press keys in the keyboard control terminal')
        print('Ctrl+C to quit')
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nKeyboard interrupt received')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
