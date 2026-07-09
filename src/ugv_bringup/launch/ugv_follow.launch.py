from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_param(name: str):
    # See astar_planner.launch.py: forces e.g. "spawn_x:=-2" to resolve as a
    # double parameter instead of an int that the node's float declaration rejects.
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('path_topic', default_value='/planned_path'),
        DeclareLaunchArgument('lookahead', default_value='0.5'),
        DeclareLaunchArgument('max_linear', default_value='1.0'),
        DeclareLaunchArgument('max_angular', default_value='2.0'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.4'),

        # Bridge: ROS Twist -> GZ (cmd_vel), GZ Odometry -> ROS (odom).
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ugv_gz_bridge',
            output='screen',
            arguments=[
                '/ugv/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/ugv/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
        ),

        Node(
            package='ugv_bringup',
            executable='pure_pursuit',
            name='ugv_pure_pursuit',
            output='screen',
            parameters=[{
                'path_topic': LaunchConfiguration('path_topic'),
                'odom_topic': '/ugv/odom',
                'cmd_topic': '/ugv/cmd_vel',
                'lookahead': _float_param('lookahead'),
                'max_linear': _float_param('max_linear'),
                'max_angular': _float_param('max_angular'),
                'goal_tolerance': _float_param('goal_tolerance'),
            }],
        ),
    ])
