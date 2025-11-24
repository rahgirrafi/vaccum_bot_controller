#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import RPi.GPIO as GPIO
import time

TB1_AIN2 = 17
TB1_AIN1 = 27
TB1_BIN1 = 22
TB1_BIN2 = 10

TB2_AIN2 = 6
TB2_AIN1 = 13
TB2_BIN1 = 19
TB2_BIN2 = 26

TB3_AIN2 = 5
TB3_AIN1 = 0
TB3_BIN1 = 11
TB3_BIN2 = 9

FRONT_RIGHT2 = TB1_AIN1
FRONT_RIGHT1 = TB1_AIN2

REAR_RIGHT1 = TB1_BIN1
REAR_RIGHT2 = TB1_BIN2

FRONT_LEFT2 = TB3_BIN1
FRONT_LEFT1 = TB3_BIN2

REAR_LEFT1 = TB3_AIN1
REAR_LEFT2 = TB3_AIN2

ARM_LEFT2 = TB2_BIN2
ARM_LEFT1 = TB2_BIN1
ARM_RIGHT1 = TB2_AIN1
ARM_RIGHT2 = TB2_AIN2

class MotorDirectionController(Node):
    """
    ROS2 node for controlling 4 motors direction using TB6612FNG drivers via Raspberry Pi GPIO pins.
    Subscribes to cmd_vel_out topic and sets direction pins based on motor velocities.
    """
    
    def __init__(self):
        super().__init__('motor_direction_controller')
        
        # GPIO pin definitions using the predefined pin constants
        # Motor 1 (Front Left)
        self.MOTOR1_INA = FRONT_LEFT1   # GPIO 5
        self.MOTOR1_INB = FRONT_LEFT2   # GPIO 0
        
        # Motor 2 (Front Right) 
        self.MOTOR2_INA = FRONT_RIGHT1  # GPIO 27
        self.MOTOR2_INB = FRONT_RIGHT2  # GPIO 17
        
        # Motor 3 (Rear Left)
        self.MOTOR3_INA = REAR_LEFT1    # GPIO 5
        self.MOTOR3_INB = REAR_LEFT2    # GPIO 0
        
        # Motor 4 (Rear Right)
        self.MOTOR4_INA = REAR_RIGHT1   # GPIO 22
        self.MOTOR4_INB = REAR_RIGHT2   # GPIO 10
        
        # Robot physical parameters (should match firmware)
        self.WHEEL_SEPARATION = 0.5  # meters between left and right wheels
        self.WHEEL_BASE = 0.4        # meters between front and rear wheels
        
        # Direction control deadzone
        self.VELOCITY_DEADZONE = 0.01  # m/s - below this, consider motor stopped
        
        # Initialize GPIO
        self.setup_gpio()
        
        # Create subscription to cmd_vel_out topic (matches firmware subscription)
        self.cmd_vel_subscription = self.create_subscription(
            TwistStamped,
            '/vaccum_base_controller/cmd_vel_out',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Motor Direction Controller Node Started')
        self.get_logger().info(f'Listening to /vaccum_base_controller/cmd_vel_out')
        self.get_logger().info(f'GPIO pins configured for 4-motor drive')
        self.get_logger().info(f'Motor pin assignments:')
        self.get_logger().info(f'  Motor 1 (FL): INA={self.MOTOR1_INA}, INB={self.MOTOR1_INB}')
        self.get_logger().info(f'  Motor 2 (FR): INA={self.MOTOR2_INA}, INB={self.MOTOR2_INB}')
        self.get_logger().info(f'  Motor 3 (RL): INA={self.MOTOR3_INA}, INB={self.MOTOR3_INB}')
        self.get_logger().info(f'  Motor 4 (RR): INA={self.MOTOR4_INA}, INB={self.MOTOR4_INB}')
        
    def setup_gpio(self):
        """Initialize GPIO pins for motor direction control"""
        try:
            # Set GPIO mode
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Configure all direction pins as outputs
            direction_pins = [
                self.MOTOR1_INA, self.MOTOR1_INB,
                self.MOTOR2_INA, self.MOTOR2_INB, 
                self.MOTOR3_INA, self.MOTOR3_INB,
                self.MOTOR4_INA, self.MOTOR4_INB
            ]
            
            for pin in direction_pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)  # Initialize to stopped state
                
            self.get_logger().info(f'GPIO pins {direction_pins} configured as outputs')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup GPIO: {e}')
            raise
            
    def set_motor_direction(self, motor_ina_pin, motor_inb_pin, velocity):
        """
        Set motor direction based on velocity using TB6612FNG truth table:
        INA=1, INB=0: Forward (CW)
        INA=0, INB=1: Reverse (CCW)  
        INA=0, INB=0: Brake/Stop
        INA=1, INB=1: Brake/Stop
        """
        try:
            if abs(velocity) < self.VELOCITY_DEADZONE:
                # Stop/Brake
                GPIO.output(motor_ina_pin, GPIO.LOW)
                GPIO.output(motor_inb_pin, GPIO.LOW)
                direction = "STOP"
            elif velocity > 0:
                # Forward
                GPIO.output(motor_ina_pin, GPIO.HIGH)
                GPIO.output(motor_inb_pin, GPIO.LOW)
                direction = "FORWARD"
            else:
                # Reverse
                GPIO.output(motor_ina_pin, GPIO.LOW)
                GPIO.output(motor_inb_pin, GPIO.HIGH)
                direction = "REVERSE"
            
            # Log pin states
            self.get_logger().debug(
                f'Motor pins {motor_ina_pin},{motor_inb_pin}: {direction} (vel={velocity:.3f})'
            )
                
        except Exception as e:
            self.get_logger().error(f'Failed to set motor direction on pins {motor_ina_pin}, {motor_inb_pin}: {e}')
    
    def cmd_vel_callback(self, msg):
        """
        Process cmd_vel messages and calculate individual motor velocities.
        For 4WD robot with differential steering, converts linear and angular velocity
        to individual wheel velocities matching the firmware's calculation.
        """
        try:
            # Extract velocities from TwistStamped message
            linear_x = msg.twist.linear.x    # m/s forward velocity
            linear_y = msg.twist.linear.y    # m/s sideways velocity (for mecanum)
            angular_z = msg.twist.angular.z  # rad/s rotational velocity
            
            # Log incoming cmd_vel for debugging
            self.get_logger().info(
                f'Received cmd_vel: linear=({linear_x:.3f}, {linear_y:.3f}), angular={angular_z:.3f}'
            )
            
            # Calculate wheel velocities (matching firmware logic)
            # For differential drive (left/right sides)
            v_left = linear_x - (angular_z * self.WHEEL_SEPARATION * 0.5)
            v_right = linear_x + (angular_z * self.WHEEL_SEPARATION * 0.5)
            
            # For 4WD, front and rear wheels on same side have same velocity
            # Motor 1: Front Left
            # Motor 2: Front Right  
            # Motor 3: Rear Left
            # Motor 4: Rear Right
            motor1_velocity = v_left   # Front Left
            motor2_velocity = v_right  # Front Right
            motor3_velocity = v_left   # Rear Left  
            motor4_velocity = v_right  # Rear Right
            
            # For mecanum wheels, add sideways component
            if abs(linear_y) > self.VELOCITY_DEADZONE:
                # Mecanum drive: sideways motion affects all wheels differently
                motor1_velocity += linear_y   # FL: + for left strafe
                motor2_velocity -= linear_y   # FR: - for left strafe  
                motor3_velocity -= linear_y   # RL: - for left strafe
                motor4_velocity += linear_y   # RR: + for left strafe
            
            # Set direction for each motor
            self.set_motor_direction(self.MOTOR1_INA, self.MOTOR1_INB, motor1_velocity)
            self.set_motor_direction(self.MOTOR2_INA, self.MOTOR2_INB, motor2_velocity) 
            self.set_motor_direction(self.MOTOR3_INA, self.MOTOR3_INB, motor3_velocity)
            self.set_motor_direction(self.MOTOR4_INA, self.MOTOR4_INB, motor4_velocity)
            
            # Log motor velocities and directions
            self.get_logger().info(
                f'Motor velocities: M1={motor1_velocity:.3f}, M2={motor2_velocity:.3f}, '
                f'M3={motor3_velocity:.3f}, M4={motor4_velocity:.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {e}')
    
    def stop_all_motors(self):
        """Emergency stop - set all motors to brake mode"""
        try:
            self.set_motor_direction(self.MOTOR1_INA, self.MOTOR1_INB, 0.0)
            self.set_motor_direction(self.MOTOR2_INA, self.MOTOR2_INB, 0.0)
            self.set_motor_direction(self.MOTOR3_INA, self.MOTOR3_INB, 0.0)
            self.set_motor_direction(self.MOTOR4_INA, self.MOTOR4_INB, 0.0)
            self.get_logger().info('All motors stopped')
        except Exception as e:
            self.get_logger().error(f'Failed to stop motors: {e}')
    
    def cleanup(self):
        """Clean shutdown - stop motors and cleanup GPIO"""
        try:
            self.get_logger().info('Cleaning up motor direction controller...')
            self.stop_all_motors()
            time.sleep(0.1)  # Give time for GPIO changes
            GPIO.cleanup()
            self.get_logger().info('GPIO cleanup completed')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')

def main(args=None):
    """Main entry point for the motor direction controller node"""
    rclpy.init(args=args)
    
    controller = None
    try:
        controller = MotorDirectionController()
        
        # Run the node
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print('\nKeyboard interrupt received, shutting down...')
    except Exception as e:
        print(f'Unexpected error: {e}')
    finally:
        # Cleanup
        if controller is not None:
            controller.cleanup()
            controller.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
