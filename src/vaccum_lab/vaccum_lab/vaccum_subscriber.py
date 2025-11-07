
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

class VaccumSubscriber(Node):
    def __init__(self):
        super().__init__('vaccum_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            'vaccum_controller/state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received JointTrajectoryControllerState message')
        # Process the message as needed
        # For example, log the current positions of the joints
        positions = msg.actual.positions
        errors_positions = msg.error.positions
        self.get_logger().info(f'Current joint positions: {positions}')