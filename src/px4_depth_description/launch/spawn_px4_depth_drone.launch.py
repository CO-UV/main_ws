from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

import math
import os


def _spawn_drone(context):
    package_share_dir = get_package_share_directory('px4_depth_description')
    default_model = os.path.join(
        package_share_dir,
        'models',
        'x500_depth_down',
        'model.sdf',
    )

    world_name = LaunchConfiguration('world_name').perform(context)
    model_file = LaunchConfiguration('model_file').perform(context) or default_model
    model_name = LaunchConfiguration('model_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = float(LaunchConfiguration('yaw').perform(context))
    yaw_half = yaw * 0.5
    qz = math.sin(yaw_half)
    qw = math.cos(yaw_half)

    request = (
        f'sdf_filename: "{model_file}" '
        f'name: "{model_name}" '
        f'pose {{ position {{ x: {x} y: {y} z: {z} }} '
        f'orientation {{ x: 0 y: 0 z: {qz} w: {qw} }} }}'
    )

    return [
        ExecuteProcess(
            cmd=[
                'gz',
                'service',
                '-s',
                f'/world/{world_name}/create',
                '--reqtype',
                'gz.msgs.EntityFactory',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                '5000',
                '--req',
                request,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    package_share_dir = get_package_share_directory('px4_depth_description')
    default_model = os.path.join(
        package_share_dir,
        'models',
        'x500_depth_down',
        'model.sdf',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='warehouse_mapping_world',
            description='Gazebo world name where the drone will be spawned.',
        ),
        DeclareLaunchArgument(
            'model_file',
            default_value=default_model,
            description='Path to the PX4 depth drone SDF model.',
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='x500_depth_down',
            description='Name of the spawned drone entity.',
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-5.5'),
        DeclareLaunchArgument('z', default_value='1.2'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_spawn_drone),
    ])
