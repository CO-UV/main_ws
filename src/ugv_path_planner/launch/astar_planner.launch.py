import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_param(name: str):
    # Launch arguments are plain strings; if the user passes e.g. "start_x:=-2"
    # (no decimal point), ROS's YAML-based parameter loader infers an integer
    # and the node's float declaration then rejects it. Forcing value_type here
    # makes both "-2" and "-2.0" resolve to a double parameter.
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    default_map_yaml = os.path.expanduser('~/maps/warehouse_occupancy.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='Saved occupancy map YAML path.',
        ),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='-10.0'),
        DeclareLaunchArgument('goal_x', default_value='9.0'),
        DeclareLaunchArgument('goal_y', default_value='9.0'),
        DeclareLaunchArgument(
            'robot_radius',
            # The planner inflates obstacles by a single circular radius, but
            # ugv_bringup's ugv_diff has a 0.9 x 0.5 m rectangular collision
            # box (half-length 0.45 m, half-width 0.25 m). 0.35 m underestimated
            # that, leaving as little as ~0.10 m real clearance on a head-on
            # approach -- observed driving the UGV's front bumper straight into
            # a shelf. 0.52 m is the box's circumscribing radius
            # (sqrt(0.45^2+0.25^2)), safe from any approach/turn angle.
            default_value='0.52',
            description='Physical UGV radius in meters (circumscribing radius of its collision box).',
        ),
        DeclareLaunchArgument(
            'obstacle_padding',
            default_value='0.20',
            description='Extra safety margin around obstacles in meters.',
        ),
        DeclareLaunchArgument(
            'clearance_radius',
            default_value='1.00',
            description='Distance around inflated obstacles where the planner adds extra cost.',
        ),
        DeclareLaunchArgument(
            'clearance_weight',
            default_value='3.00',
            description='Cost weight for paths close to inflated obstacles.',
        ),
        DeclareLaunchArgument(
            'unknown_is_occupied',
            default_value='true',
            description='Treat unknown map cells as blocked.',
        ),
        Node(
            package='ugv_path_planner',
            executable='astar_planner',
            name='astar_planner',
            output='screen',
            parameters=[{
                'map_yaml': LaunchConfiguration('map_yaml'),
                'start_x': _float_param('start_x'),
                'start_y': _float_param('start_y'),
                'goal_x': _float_param('goal_x'),
                'goal_y': _float_param('goal_y'),
                'robot_radius': _float_param('robot_radius'),
                'obstacle_padding': _float_param('obstacle_padding'),
                'clearance_radius': _float_param('clearance_radius'),
                'clearance_weight': _float_param('clearance_weight'),
                'unknown_is_occupied': LaunchConfiguration('unknown_is_occupied'),
            }],
        ),
    ])
