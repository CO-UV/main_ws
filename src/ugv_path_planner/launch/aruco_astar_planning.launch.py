from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('ugv_path_planner'),
        'rviz',
        'aruco_astar.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value='~/main_ws/maps/warehouse_map.yaml'),
        DeclareLaunchArgument('goal_yaml', default_value='~/main_ws/maps/aruco_marker.yaml'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='-5.5'),
        DeclareLaunchArgument('goal_standoff_distance', default_value='0.5'),
        DeclareLaunchArgument('robot_radius', default_value='0.20'),
        DeclareLaunchArgument('obstacle_padding', default_value='0.01'),
        DeclareLaunchArgument('clearance_radius', default_value='0.30'),
        DeclareLaunchArgument('clearance_weight', default_value='0.80'),
        DeclareLaunchArgument('unknown_is_occupied', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='ugv_path_planner',
            executable='astar_planner',
            name='saved_aruco_astar_planner',
            output='screen',
            parameters=[{
                'use_map_topic': False,
                'map_yaml': LaunchConfiguration('map_yaml'),
                'frame_id': LaunchConfiguration('frame_id'),
                'start_x': LaunchConfiguration('start_x'),
                'start_y': LaunchConfiguration('start_y'),
                'use_goal_file': True,
                'goal_yaml': LaunchConfiguration('goal_yaml'),
                'goal_standoff_distance': LaunchConfiguration('goal_standoff_distance'),
                'robot_radius': LaunchConfiguration('robot_radius'),
                'obstacle_padding': LaunchConfiguration('obstacle_padding'),
                'clearance_radius': LaunchConfiguration('clearance_radius'),
                'clearance_weight': LaunchConfiguration('clearance_weight'),
                'unknown_is_occupied': LaunchConfiguration('unknown_is_occupied'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='aruco_astar_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
