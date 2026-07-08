from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('warehouse_mapping')
    slam_mapping_launch = os.path.join(
        package_share_dir,
        'launch',
        'slam_mapping.launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_mapping_launch),
        ),
    ])
