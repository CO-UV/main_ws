from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    default_model = os.path.join(
        get_package_share_directory('ugv_bringup'),
        'models', 'ugv_diff', 'model.sdf',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='warehouse'),
        DeclareLaunchArgument('model_file', default_value=default_model),
        DeclareLaunchArgument('name', default_value='ugv_diff'),
        # Spawn at the A* path start, facing +x (yaw=0) so odom == world axes.
        DeclareLaunchArgument('x', default_value='-9.17'),
        DeclareLaunchArgument('y', default_value='-5.57'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_ugv',
            output='screen',
            arguments=[
                '-world', LaunchConfiguration('world'),
                '-file', LaunchConfiguration('model_file'),
                '-name', LaunchConfiguration('name'),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        ),
    ])
