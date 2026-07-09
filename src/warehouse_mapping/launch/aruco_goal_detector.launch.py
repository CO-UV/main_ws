from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/rgb_camera'),
        DeclareLaunchArgument('camera_info_topic', default_value='/rgb_camera/camera_info'),
        DeclareLaunchArgument('marker_id', default_value='23'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('goal_topic', default_value='/aruco/goal_pose'),
        DeclareLaunchArgument('save_pose_path', default_value='~/main_ws/maps/aruco_marker.yaml'),
        DeclareLaunchArgument('filter_window_size', default_value='40'),
        DeclareLaunchArgument('min_stable_samples', default_value='30'),
        DeclareLaunchArgument('max_position_std', default_value='0.20'),
        Node(
            package='warehouse_mapping',
            executable='aruco_goal_detector',
            name='aruco_goal_detector',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'marker_id': LaunchConfiguration('marker_id'),
                'target_frame': LaunchConfiguration('target_frame'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'save_pose_path': LaunchConfiguration('save_pose_path'),
                'save_pose': True,
                'filter_window_size': LaunchConfiguration('filter_window_size'),
                'min_stable_samples': LaunchConfiguration('min_stable_samples'),
                'max_position_std': LaunchConfiguration('max_position_std'),
            }],
        ),
    ])
