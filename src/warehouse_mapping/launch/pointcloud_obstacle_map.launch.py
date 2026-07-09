from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/rtabmap/cloud_map'),
        DeclareLaunchArgument('map_topic', default_value='/pointcloud_obstacle_map'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('min_x', default_value='-13.5'),
        DeclareLaunchArgument('max_x', default_value='13.5'),
        DeclareLaunchArgument('min_y', default_value='-11.5'),
        DeclareLaunchArgument('max_y', default_value='11.5'),
        DeclareLaunchArgument('min_z', default_value='0.15'),
        DeclareLaunchArgument('max_z', default_value='3.2'),
        DeclareLaunchArgument('point_radius', default_value='0.01'),
        DeclareLaunchArgument('inflation_radius', default_value='0.00'),
        DeclareLaunchArgument('min_points_per_cell', default_value='2'),
        DeclareLaunchArgument('hit_count_threshold', default_value='8'),
        DeclareLaunchArgument('accumulate', default_value='true'),
        Node(
            package='warehouse_mapping',
            executable='pointcloud_to_occupancy_grid',
            name='pointcloud_to_occupancy_grid',
            output='screen',
            parameters=[{
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'map_topic': LaunchConfiguration('map_topic'),
                'target_frame': LaunchConfiguration('target_frame'),
                'resolution': LaunchConfiguration('resolution'),
                'min_x': LaunchConfiguration('min_x'),
                'max_x': LaunchConfiguration('max_x'),
                'min_y': LaunchConfiguration('min_y'),
                'max_y': LaunchConfiguration('max_y'),
                'min_z': LaunchConfiguration('min_z'),
                'max_z': LaunchConfiguration('max_z'),
                'point_radius': LaunchConfiguration('point_radius'),
                'inflation_radius': LaunchConfiguration('inflation_radius'),
                'min_points_per_cell': LaunchConfiguration('min_points_per_cell'),
                'hit_count_threshold': LaunchConfiguration('hit_count_threshold'),
                'accumulate': LaunchConfiguration('accumulate'),
            }],
        ),
    ])
