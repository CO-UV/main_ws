from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    warehouse_share_dir = get_package_share_directory('warehouse_mapping')
    px4_depth_share_dir = get_package_share_directory('px4_depth_description')

    gazebo_launch = os.path.join(
        warehouse_share_dir,
        'launch',
        'gazebo_warehouse.launch.py',
    )
    spawn_launch = os.path.join(
        px4_depth_share_dir,
        'launch',
        'spawn_px4_depth_drone.launch.py',
    )
    slam_launch = os.path.join(
        warehouse_share_dir,
        'launch',
        'slam_mapping.launch.py',
    )
    offboard_launch = os.path.join(
        warehouse_share_dir,
        'launch',
        'offboard_path.launch.py',
    )

    start_agent = LaunchConfiguration('start_agent')
    start_px4 = LaunchConfiguration('start_px4')
    start_offboard = LaunchConfiguration('start_offboard')
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    px4_dir = LaunchConfiguration('px4_dir')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value='-v 2 -r',
            description='Gazebo args. Default opens the GUI.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='warehouse',
            description='Gazebo world name used by spawn and PX4.',
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='x500_depth_down',
            description='Spawned Gazebo model name for PX4 to attach to.',
        ),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=os.path.expanduser('~/PX4-Autopilot'),
            description='PX4-Autopilot directory.',
        ),
        DeclareLaunchArgument(
            'px4_target',
            default_value='gz_x500',
            description='PX4 SITL make target.',
        ),
        DeclareLaunchArgument(
            'start_agent',
            default_value='true',
            description='Start Micro XRCE-DDS Agent.',
        ),
        DeclareLaunchArgument(
            'start_px4',
            default_value='false',
            description='Start PX4 SITL and attach it to the spawned model.',
        ),
        DeclareLaunchArgument(
            'start_offboard',
            default_value='false',
            description='Start offboard path flight after SLAM is running.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for ROS nodes.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Launch RTAB-Map visualization UI.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz from rtabmap_launch.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': LaunchConfiguration('gz_args'),
            }.items(),
        ),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            condition=IfCondition(start_agent),
        ),
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_launch),
                    launch_arguments={
                        'world_name': world_name,
                        'model_name': model_name,
                    }.items(),
                ),
            ],
        ),
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'make',
                        'px4_sitl',
                        LaunchConfiguration('px4_target'),
                    ],
                    cwd=px4_dir,
                    output='screen',
                    additional_env={
                        'PX4_GZ_STANDALONE': '1',
                        'PX4_GZ_WORLD': world_name,
                        'PX4_GZ_MODEL_NAME': model_name,
                        'PX4_HOME_LAT': '37.5665',
                        'PX4_HOME_LON': '126.9780',
                        'PX4_HOME_ALT': '0',
                    },
                    condition=IfCondition(start_px4),
                ),
            ],
        ),
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                        'rviz': LaunchConfiguration('rviz'),
                    }.items(),
                ),
            ],
        ),
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(offboard_launch),
                    condition=IfCondition(start_offboard),
                ),
            ],
        ),
    ])
