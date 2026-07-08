from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('px4_depth_description')
    urdf_path = os.path.join(
        package_share_dir,
        'urdf',
        'x500_depth_down_tf.urdf',
    )

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='x500_depth_down_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description,
                'publish_frequency': 30.0,
            }],
        ),
    ])
