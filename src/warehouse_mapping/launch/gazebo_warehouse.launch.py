from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('warehouse_mapping')
    px4_depth_share_dir = get_package_share_directory('px4_depth_description')
    default_world = os.path.join(package_share_dir, 'worlds', 'ugv_warehouse.sdf')
    warehouse_model_path = os.path.join(package_share_dir, 'models')
    local_model_path = os.path.join(px4_depth_share_dir, 'models')
    px4_model_path = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')
    gz_fuel_paths = [
        os.path.expanduser('~/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models'),
        os.path.expanduser('~/.gz/fuel/fuel.ignitionrobotics.org/movai/models'),
    ]
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to the Gazebo warehouse SDF world.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value='-v 2 -r -s',
            description='Extra arguments passed to gz sim.',
        ),
        ExecuteProcess(
            cmd=['gz sim ', world, ' ', gz_args],
            shell=True,
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': ':'.join(
                    [warehouse_model_path, local_model_path, px4_model_path] + gz_fuel_paths
                ),
            },
        ),
    ])
