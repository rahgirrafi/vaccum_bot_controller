from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    ARGUMENTS = [
    DeclareLaunchArgument('use_sim', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation (Gazebo) or hardware'),

]
    vaccum_desc_pkg_share = get_package_share_directory('vaccum_description')

    control_pkg_share = get_package_share_directory('vaccum_control')
    vaccum_gz_pkg_share = get_package_share_directory('vaccum_gz')
    
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation (Gazebo) or hardware'
    )

    spawn_entity_launch = PathJoinSubstitution(
        [vaccum_desc_pkg_share, 'launch', 'spawn.launch.py']
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_entity_launch),
    )

    rviz_config_file = PathJoinSubstitution(
        [vaccum_gz_pkg_share, 'config', 'vaccum_default.rviz']
    )


    controllers = PathJoinSubstitution(
        [control_pkg_share, 'config', 'controllers.yaml']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers],
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description')
        ]
        
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["vaccum_base_controller", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(declare_use_sim)
    ld.add_action(spawn_entity)
    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_controller_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(rviz)
    return ld