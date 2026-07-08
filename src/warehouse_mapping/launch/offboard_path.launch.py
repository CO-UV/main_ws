from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'altitude',
            default_value='3.5',
            description='Flight altitude above the Gazebo floor in meters.',
        ),
        DeclareLaunchArgument(
            'auto_arm',
            default_value='true',
            description='Automatically arm the vehicle after entering offboard mode.',
        ),
        DeclareLaunchArgument(
            'auto_land',
            default_value='true',
            description='Automatically land when the path is complete.',
        ),
        DeclareLaunchArgument(
            'acceptance_radius',
            default_value='0.80',
            description='Waypoint acceptance radius in meters.',
        ),
        DeclareLaunchArgument(
            'hold_time_s',
            default_value='0.0',
            description='Seconds to hold each waypoint after arrival.',
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='0.15',
            description='Requested PX4 ground speed in meters per second.',
        ),
        DeclareLaunchArgument(
            'local_position_topic',
            default_value='/fmu/out/vehicle_local_position_v1',
            description='PX4 local position topic. Some PX4 versions publish only the _v1 name.',
        ),
        DeclareLaunchArgument(
            'status_topic',
            default_value='/fmu/out/vehicle_status',
            description='PX4 vehicle status topic.',
        ),
        DeclareLaunchArgument(
            'require_status',
            default_value='false',
            description='Wait for vehicle_status to confirm offboard and armed before moving.',
        ),
        Node(
            package='warehouse_mapping',
            executable='offboard_path',
            name='warehouse_offboard_path',
            output='screen',
            parameters=[{
                'altitude': LaunchConfiguration('altitude'),
                'auto_arm': LaunchConfiguration('auto_arm'),
                'auto_land': LaunchConfiguration('auto_land'),
                'acceptance_radius': LaunchConfiguration('acceptance_radius'),
                'hold_time_s': LaunchConfiguration('hold_time_s'),
                'max_speed': LaunchConfiguration('max_speed'),
                'local_position_topic': LaunchConfiguration('local_position_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
                'require_status': LaunchConfiguration('require_status'),
            }],
        ),
    ])
