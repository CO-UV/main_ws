from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value='~/main_ws/maps/warehouse_map.yaml',
            description='Saved occupancy map YAML path.',
        ),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='-5.5'),
        DeclareLaunchArgument('goal_x', default_value='9.5'),
        DeclareLaunchArgument('goal_y', default_value='8.2'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('use_goal_file', default_value='false'),
        DeclareLaunchArgument('goal_yaml', default_value='~/main_ws/maps/aruco_marker.yaml'),
        DeclareLaunchArgument('goal_standoff_distance', default_value='0.5'),
        DeclareLaunchArgument(
            'robot_radius',
            default_value='0.20',
            description='Physical UGV radius in meters.',
        ),
        DeclareLaunchArgument(
            'obstacle_padding',
            default_value='0.01',
            description='Extra safety margin around obstacles in meters.',
        ),
        DeclareLaunchArgument(
            'clearance_radius',
            default_value='0.30',
            description='Distance around inflated obstacles where the planner adds extra cost.',
        ),
        DeclareLaunchArgument(
            'clearance_weight',
            default_value='0.80',
            description='Cost weight for paths close to inflated obstacles.',
        ),
        DeclareLaunchArgument(
            'unknown_is_occupied',
            default_value='false',
            description='Treat unknown map cells as blocked.',
        ),
        Node(
            package='ugv_path_planner',
            executable='astar_planner',
            name='astar_planner',
            output='screen',
            parameters=[{
                'map_yaml': LaunchConfiguration('map_yaml'),
                'start_x': LaunchConfiguration('start_x'),
                'start_y': LaunchConfiguration('start_y'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'frame_id': LaunchConfiguration('frame_id'),
                'use_goal_file': LaunchConfiguration('use_goal_file'),
                'goal_yaml': LaunchConfiguration('goal_yaml'),
                'goal_standoff_distance': LaunchConfiguration('goal_standoff_distance'),
                'robot_radius': LaunchConfiguration('robot_radius'),
                'obstacle_padding': LaunchConfiguration('obstacle_padding'),
                'clearance_radius': LaunchConfiguration('clearance_radius'),
                'clearance_weight': LaunchConfiguration('clearance_weight'),
                'unknown_is_occupied': LaunchConfiguration('unknown_is_occupied'),
            }],
        ),
    ])
