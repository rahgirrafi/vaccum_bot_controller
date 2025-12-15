
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from custom_interfaces.msg import Float32FixedArray8
import numpy as np

class VaccumSubscriber(Node):
    def __init__(self):
        super().__init__('vaccum_subscriber')
        
        # Subscriber to joint trajectory controller state
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Publisher for Float32FixedArray8
        self.publisher = self.create_publisher(
            Float32FixedArray8,
            '/joint_state_array',
            10)
        
        self.get_logger().info('VaccumSubscriber node started')

    def listener_callback(self, msg):
        # Get actual joint error
        positions = msg.error.positions

        # Create Float32FixedArray8 message
        array_msg = Float32FixedArray8()
        
        # Fill the 8-element array with joint data
        # If less than 8 joints, pad with zeros; if more, truncate
        array_msg.element = [0.0] * 8
        for i in range(min(len(positions), 8)):
            error = float(positions[i])
            # Map error to motor direction: 1, -1, or 0
            if abs(error) < 0.001:
                direction = 0.0
            else:
                if error > 0:
                    direction = 1.0
                elif error < 0:
                    direction = -1.0
                else:
                    direction = 0.0
            array_msg.element[i] = direction
        
        # Publish the array
        self.publisher.publish(array_msg)
        
        self.get_logger().info(f'Published motor directions: {array_msg.element}')
        
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