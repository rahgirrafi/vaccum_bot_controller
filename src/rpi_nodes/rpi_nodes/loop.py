#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import time

class LoopControlNode(Node):
    """
    Simple loop node that continuously publishes to group3 and group4
    alternating between 't' (value=1) and 'v' (value=-1) every 1 second.
    
    This simulates pressing 't' key (group4=1) then 'v' key (group3=-1) repeatedly.
    """
    
    def __init__(self):
        super().__init__('loop_control_node')
        
        # Create publishers for group3 and group4
        self.group3_publisher = self.create_publisher(Int8, '/keyboard/group3', 10)
        self.group4_publisher = self.create_publisher(Int8, '/keyboard/group4', 10)
        
        # Timer to send commands every 1 second
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # State tracker to alternate between 't' and 'v'
        self.send_t = True
        
        self.get_logger().info('Loop Control Node Started')
        self.get_logger().info('Publishing t (group4=1) and v (group3=-1) every 1 second')
        self.get_logger().info('Press Ctrl+C to stop')
        
    def timer_callback(self):
        """Timer callback that alternates between 't' and 'v' commands"""
        try:
            if self.send_t:
                # Send 't' - publishes to group4 with value 1
                msg = Int8()
                msg.data = 1
                self.group4_publisher.publish(msg)
                self.get_logger().info('Sent: t -> group4 = 1')
            else:
                # Send 'v' - publishes to group3 with value -1
                msg = Int8()
                msg.data = -1
                self.group4_publisher.publish(msg)
                self.get_logger().info('Sent: v -> group3 = -1')
            
            # Toggle state for next iteration
            self.send_t = not self.send_t
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')


def main(args=None):
    """Main entry point for the loop control node"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = LoopControlNode()
        
        # Run the node (will loop indefinitely)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\nCtrl+C pressed, shutting down...')
    except Exception as e:
        print(f'Unexpected error: {e}')
    finally:
        # Cleanup
        if node is not None:
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
