import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('acc_gazebo_sim')
    world = LaunchConfiguration('world')
    world_default = PathJoinSubstitution([pkg_share, 'worlds', 'acc_world.sdf'])
    bridge_config = os.path.join(
        get_package_share_directory('acc_gazebo_sim'), 'config', 'bridge.yaml'
    )
    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_default,
                              description='SDF world file to start in Gazebo Sim'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'gz_args': ['-r ', world]}.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_ros_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
            output='screen',
        ),
        Node(
            package='acc_gazebo_sim',
            executable='gazebo_acc_controller',
            name='gazebo_acc_controller',
            output='screen',
        ),
    ])
