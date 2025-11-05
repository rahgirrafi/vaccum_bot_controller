from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    ign_pkg_share = get_package_share_directory('ros_gz_sim')
    vaccum_desc_pkg_share = get_package_share_directory('vaccum_description')
    control_pkg_share = get_package_share_directory('vaccum_control')
    vaccum_gz_pkg_share = get_package_share_directory('vaccum_gz')
    gz_args = LaunchConfiguration('gz_args', default='')
    rviz_config_file = PathJoinSubstitution(
        [vaccum_gz_pkg_share, 'config', 'vaccum_default.rviz']
    )
    gz_sim_launch = PathJoinSubstitution(
        [ign_pkg_share, 'launch', 'gz_sim.launch.py']
    )

    spawn_entity_launch = PathJoinSubstitution(
        [vaccum_desc_pkg_share, 'launch', 'spawn.launch.py']
    )


    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image'],
        output='screen'
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
        output='screen'
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'
    )


    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, f' -r -v 1 {vaccum_gz_pkg_share}/worlds/warehouse.sdf'])])



    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_entity_launch),
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

    return LaunchDescription([
        bridge,
        camera_bridge,
        camera_info_bridge,
        lidar_bridge,
        gazebo,
        spawn_entity,
        # control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        arm_controller_spawner,
        rviz,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])