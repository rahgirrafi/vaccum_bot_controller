#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for the keyboard control node.
    This node reads keyboard input and publishes to 4 different topics.
    """
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for the keyboard control node'
    )
    
    # Keyboard control node
    keyboard_control_node = Node(
        package='rpi_nodes',
        executable='keyboard_control',
        name='keyboard_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        log_level_arg,
        keyboard_control_node,
    ])
