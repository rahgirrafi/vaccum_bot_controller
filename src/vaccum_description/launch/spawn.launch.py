from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():



    robot_desc = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), 
            ' ', 
            PathJoinSubstitution(
                [FindPackageShare('vaccum_description'), 'urdf', 'vaccum.urdf.xacro']
            ), 
         ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'vaccum', '-allow_renaming', 'true', '-x', '0.0', '-y', '0.0', '-z', '0.8'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity)
    # ld.add_action(joint_trajectory_controller_spawner)

    return ld