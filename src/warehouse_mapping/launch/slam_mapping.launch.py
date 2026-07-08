from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('warehouse_mapping')
    bridge_launch = os.path.join(
        package_share_dir,
        'launch',
        'depth_camera_bridge.launch.py',
    )
    px4_depth_share_dir = get_package_share_directory('px4_depth_description')
    camera_tf_launch = os.path.join(
        px4_depth_share_dir,
        'launch',
        'camera_tf.launch.py',
    )
    rtabmap_launch = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='true',
            description='Launch RTAB-Map visualization UI.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz from rtabmap_launch.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_tf_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        Node(
            package='warehouse_mapping',
            executable='px4_odometry_bridge',
            name='px4_odometry_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'local_position_topic': '/fmu/out/vehicle_local_position_v1'},
                {'odom_topic': '/px4/odom'},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_link'},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'depth': 'false',
                'subscribe_rgb': 'false',
                'subscribe_rgbd': 'false',
                'subscribe_scan_cloud': 'true',
                'scan_cloud_topic': '/depth_camera/points',
                'visual_odometry': 'false',
                'icp_odometry': 'false',
                'odom_topic': '/px4/odom',
                'frame_id': 'base_link',
                'map_topic': '/map',
                'qos_scan': '1',
                'qos_odom': '1',
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'log_level': 'info',
            }.items(),
        ),
    ])
