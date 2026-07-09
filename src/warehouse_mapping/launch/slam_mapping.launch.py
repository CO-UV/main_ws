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
            default_value='false',
            description='Use simulation time.',
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
                {'origin_x': 0.0},
                {'origin_y': -5.5},
            ],
        ),
        Node(
            package='warehouse_mapping',
            executable='pointcloud_timestamp_republisher',
            name='depth_points_timestamp_sync',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'input_topic': '/depth_camera/points'},
                {'output_topic': '/depth_camera/points_synced'},
                {'frame_id': 'camera_link'},
                {'enable_altitude_gate': True},
                {'local_position_topic': '/fmu/out/vehicle_local_position_v1'},
                {'target_altitude': 4.2},
                {'altitude_tolerance': 0.35},
                {'stable_time_s': 1.5},
                {'enable_mapping_active_gate': True},
                {'mapping_active_topic': '/mapping/active'},
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
                'scan_cloud_topic': '/depth_camera/points_synced',
                'visual_odometry': 'false',
                'icp_odometry': 'false',
                'odom_topic': '/px4/odom',
                'frame_id': 'base_link',
                'map_topic': '/map',
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.5',
                'qos_scan': '2',
                'qos_odom': '1',
                'args': (
                    '--delete_db_on_start '
                    '--Grid/FromDepth false '
                    '--Grid/3D true '
                    '--Grid/RayTracing true '
                    '--Grid/RangeMax 7.0 '
                    '--Grid/CellSize 0.05 '
                    '--Grid/PreVoxelFiltering true '
                    '--Grid/MapFrameProjection true '
                    '--Grid/NormalsSegmentation false '
                    '--Grid/MaxGroundHeight 0.12 '
                    '--Grid/MaxObstacleHeight 2.20 '
                    '--Grid/ClusterRadius 0.10 '
                    '--Grid/MinClusterSize 15 '
                    '--Grid/NoiseFilteringRadius 0.10 '
                    '--Grid/NoiseFilteringMinNeighbors 6 '
                    '--GridGlobal/OccupancyThr 0.45 '
                    '--GridGlobal/ProbHit 0.78 '
                    '--GridGlobal/ProbMiss 0.48'
                ),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'log_level': 'info',
            }.items(),
        ),
    ])
