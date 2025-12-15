
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from custom_interfaces.msg import Float32FixedArray4
import sys
import tty
import termios
import select
import time

class VaccumSubscriber(Node):
    def __init__(self):
        super().__init__('vaccum_subscriber')
             
        # Publisher for Float32FixedArray4
        self.publisher = self.create_publisher(
            Float32FixedArray4,
            '/joint_state_array',
            10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Keyboard mapping: key -> (index, value)
        self.key_mapping = {
            'q': (0, 1.0),  'a': (0, 0.0),  'z': (0, -1.0),
            'w': (1, 1.0),  's': (1, 0.0),  'x': (1, -1.0),
            'e': (2, 1.0),  'd': (2, 0.0),  'c': (2, -1.0),
            'r': (3, 1.0),  'f': (3, 0.0),  'v': (3, -1.0)
        }
        
        # Current motor directions (4 elements)
        self.motor_directions = [0.0, 0.0, 0.0, 0.0]
        
        # Track last key press time
        self.last_key_time = time.time()
        self.timeout = 0.5  # 0.5 seconds timeout
        
        # Terminal settings for non-blocking keyboard input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        self.get_logger().info('Keyboard Control node started')
        self.get_logger().info('Controls: [q,a,z], [w,s,x], [e,d,c], [r,f,v]')
        self.get_logger().info('Press Ctrl+C to exit')

    def get_key(self):
        """Non-blocking keyboard input"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def timer_callback(self):
        # Check for keyboard input
        key = self.get_key()
        if key and key in self.key_mapping:
            index, value = self.key_mapping[key]
            self.motor_directions[index] = value
            self.last_key_time = time.time()
            self.get_logger().info(f'Key "{key}" pressed -> motor[{index}] = {value}')
        
        # Check if timeout has passed since last key press
        if time.time() - self.last_key_time > self.timeout:
            # Reset all motor directions to zero
            if any(d != 0.0 for d in self.motor_directions):
                self.motor_directions = [0.0, 0.0, 0.0, 0.0]
                self.get_logger().info('Timeout reached - all motors set to 0')
        
        # Create Float32FixedArray4 message
        array_msg = Float32FixedArray4()
        array_msg.element = self.motor_directions.copy()
        
        # Publish the array
        self.publisher.publish(array_msg)
        
    def __del__(self):
        """Restore terminal settings on exit"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    rclpy.init(args=args)
    vaccum_subscriber = VaccumSubscriber()
    
    try:
        rclpy.spin(vaccum_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        vaccum_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()