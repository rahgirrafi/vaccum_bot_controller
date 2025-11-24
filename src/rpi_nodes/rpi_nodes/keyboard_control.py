#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import sys
import select
import termios
import tty
import threading
import time
import RPi.GPIO as GPIO

TB2_AIN2 = 6
TB2_AIN1 = 13
TB2_BIN1 = 19
TB2_BIN2 = 26

ARM_LEFT2 = TB2_BIN2
ARM_LEFT1 = TB2_BIN1
ARM_RIGHT1 = TB2_AIN1
ARM_RIGHT2 = TB2_AIN2


class KeyboardControlNode(Node):
    """
    ROS2 node for reading keyboard input and publishing to 4 different topics.
    Also controls motor direction pins for groups 2 and 3.
    
    Key groups:
    - (q,a,z) -> /keyboard/group1 topic (no motor control)
    - (w,s,x) -> /keyboard/group2 topic + Motor A control (TB2_AIN1, TB2_AIN2)
    - (r,f,v) -> /keyboard/group3 topic + Motor B control (TB2_BIN1, TB2_BIN2)
    - (t,g,b) -> /keyboard/group4 topic (no motor control)
    
    For each group: first key = 1, middle key = 0, last key = -1
    Motor control: 1=Forward, 0=Stop, -1=Reverse
    """
    
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # Key mappings for each group
        self.key_groups = {
            'q': {'topic': 'group1', 'value': 1},
            'a': {'topic': 'group1', 'value': 0},
            'z': {'topic': 'group1', 'value': -1},
            
            'w': {'topic': 'group2', 'value': 1},
            's': {'topic': 'group2', 'value': 0},
            'x': {'topic': 'group2', 'value': -1},
            
            'r': {'topic': 'group3', 'value': 1},
            'f': {'topic': 'group3', 'value': 0},
            'v': {'topic': 'group3', 'value': -1},
            
            't': {'topic': 'group4', 'value': 1},
            'g': {'topic': 'group4', 'value': 0},
            'b': {'topic': 'group4', 'value': -1},
        }
        
        # Create publishers for each group
        self.group_publishers = {
            'group1': self.create_publisher(Int8, '/keyboard/group1', 10),
            'group2': self.create_publisher(Int8, '/keyboard/group2', 10),
            'group3': self.create_publisher(Int8, '/keyboard/group3', 10),
            'group4': self.create_publisher(Int8, '/keyboard/group4', 10),
        }
        
        # Terminal settings for raw keyboard input
        self.old_settings = None
        self.running = True
        
        # Initialize GPIO for motor control
        self.setup_gpio()
        
        # Setup terminal for raw input
        self.setup_terminal()
        
        # Start keyboard reading thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_reader, daemon=True)
        self.keyboard_thread.start()
        
        # Create a timer to keep the node alive and publish periodic status
        self.status_timer = self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('Keyboard Control Node Started with Motor Control')
        self.get_logger().info('Key mappings:')
        self.get_logger().info('  Group 1 (q=1, a=0, z=-1) -> /keyboard/group1 (no motor)')
        self.get_logger().info('  Group 2 (w=1, s=0, x=-1) -> /keyboard/group2 + Motor A')
        self.get_logger().info('  Group 3 (r=1, f=0, v=-1) -> /keyboard/group3 + Motor B')
        self.get_logger().info('  Group 4 (t=1, g=0, b=-1) -> /keyboard/group4 (no motor)')
        self.get_logger().info('Motor Control: 1=Forward, 0=Stop, -1=Reverse')
        self.get_logger().info('Press ESC or Ctrl+C to quit')
        
    def setup_gpio(self):
        """Initialize GPIO pins for motor direction control"""
        try:
            # Set GPIO mode
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Configure TB2 pins as outputs
            motor_pins = [TB2_AIN1, TB2_AIN2, TB2_BIN1, TB2_BIN2]
            
            for pin in motor_pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)  # Initialize to stopped state
                
            self.get_logger().info(f'GPIO pins {motor_pins} configured for motor control')
            self.get_logger().info(f'TB2_AIN1={TB2_AIN1}, TB2_AIN2={TB2_AIN2}, TB2_BIN1={TB2_BIN1}, TB2_BIN2={TB2_BIN2}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup GPIO: {e}')
            raise
    
    def set_motor_direction(self, motor_a_pin, motor_b_pin, direction):
        """
        Set motor direction using TB6612FNG truth table:
        direction = 1:  INA=1, INB=0 (Forward)
        direction = -1: INA=0, INB=1 (Reverse)  
        direction = 0:  INA=0, INB=0 (Stop/Brake)
        """
        try:
            if direction == 1:
                # Forward
                GPIO.output(motor_a_pin, GPIO.HIGH)
                GPIO.output(motor_b_pin, GPIO.LOW)
                dir_str = "FORWARD"
            elif direction == -1:
                # Reverse
                GPIO.output(motor_a_pin, GPIO.LOW)
                GPIO.output(motor_b_pin, GPIO.HIGH)
                dir_str = "REVERSE"
            else:
                # Stop/Brake
                GPIO.output(motor_a_pin, GPIO.LOW)
                GPIO.output(motor_b_pin, GPIO.LOW)
                dir_str = "STOP"
            
            self.get_logger().info(f'Motor pins {motor_a_pin},{motor_b_pin}: {dir_str}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to set motor direction on pins {motor_a_pin}, {motor_b_pin}: {e}')
        
    def setup_terminal(self):
        """Setup terminal for raw keyboard input"""
        try:
            if sys.stdin.isatty():
                self.old_settings = termios.tcgetattr(sys.stdin)
                tty.setraw(sys.stdin.fileno())
                self.get_logger().info('Terminal setup for raw keyboard input')
            else:
                self.get_logger().warn('Not running in a terminal, keyboard input may not work')
        except Exception as e:
            self.get_logger().error(f'Failed to setup terminal: {e}')
    
    def restore_terminal(self):
        """Restore terminal settings"""
        try:
            if self.old_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                self.get_logger().info('Terminal settings restored')
        except Exception as e:
            self.get_logger().error(f'Failed to restore terminal: {e}')
    
    def keyboard_reader(self):
        """Thread function to read keyboard input"""
        try:
            while self.running and rclpy.ok():
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.process_key(key)
                time.sleep(0.01)  # Small delay to prevent high CPU usage
                
        except Exception as e:
            self.get_logger().error(f'Error in keyboard reader: {e}')
    
    def process_key(self, key):
        """Process a pressed key and publish to appropriate topic"""
        try:
            # Check for escape key (quit)
            if ord(key) == 27:  # ESC key
                self.get_logger().info('ESC pressed, shutting down...')
                self.running = False
                rclpy.shutdown()
                return
            
            # Convert to lowercase for case-insensitive matching
            key = key.lower()
            
            # Check if key is in our mapping
            if key in self.key_groups:
                group_info = self.key_groups[key]
                topic_name = group_info['topic']
                value = group_info['value']
                
                # Create and publish message
                msg = Int8()
                msg.data = value
                
                self.group_publishers[topic_name].publish(msg)
                
                # Control motors for groups 2 and 3
                if topic_name == 'group2':
                    # Group 2 controls Motor A (TB2_AIN1, TB2_AIN2)
                    self.set_motor_direction(TB2_AIN1, TB2_AIN2, value)
                elif topic_name == 'group3':
                    # Group 3 controls Motor B (TB2_BIN1, TB2_BIN2)  
                    self.set_motor_direction(TB2_BIN1, TB2_BIN2, value)
                
                # Log the action
                self.get_logger().info(
                    f'Key "{key}" pressed -> {topic_name}: {value}'
                )
                
            else:
                # Log unrecognized keys for debugging
                self.get_logger().debug(f'Unrecognized key: "{key}" (ord: {ord(key)})')
                
        except Exception as e:
            self.get_logger().error(f'Error processing key "{key}": {e}')
    
    def status_callback(self):
        """Periodic status callback to show node is alive"""
        self.get_logger().debug('Keyboard control node is running...')
    
    def cleanup(self):
        """Clean shutdown"""
        try:
            self.get_logger().info('Cleaning up keyboard control node...')
            self.running = False
            
            # Stop all motors before cleanup
            self.set_motor_direction(TB2_AIN1, TB2_AIN2, 0)  # Stop Motor A
            self.set_motor_direction(TB2_BIN1, TB2_BIN2, 0)  # Stop Motor B
            
            # Restore terminal
            self.restore_terminal()
            
            # Wait a bit for the thread to finish
            if self.keyboard_thread.is_alive():
                self.keyboard_thread.join(timeout=1.0)
            
            # Cleanup GPIO
            GPIO.cleanup()
            self.get_logger().info('GPIO cleanup completed')
                
            self.get_logger().info('Keyboard control node cleanup completed')
            
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')


def get_key_non_blocking():
    """Get a single keypress without blocking (alternative method)"""
    try:
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
    except:
        pass
    return None


def main(args=None):
    """Main entry point for the keyboard control node"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = KeyboardControlNode()
        
        # Run the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\nKeyboard interrupt received, shutting down...')
    except Exception as e:
        print(f'Unexpected error: {e}')
    finally:
        # Cleanup
        if node is not None:
            node.cleanup()
            node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
