import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_mapping'


def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]
        if files:
            paths.append((os.path.join('share', package_name, path), files))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ] + package_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hong',
    maintainer_email='hong@todo.todo',
    description='Bringup and configuration package for PX4 warehouse mapping simulation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_odometry_bridge = warehouse_mapping.px4_odometry_bridge:main',
            'offboard_path = warehouse_mapping.offboard_path:main',
            'pointcloud_timestamp_republisher = warehouse_mapping.pointcloud_timestamp_republisher:main',
            'aruco_goal_detector = warehouse_mapping.aruco_goal_detector:main',
            'save_occupancy_map = warehouse_mapping.save_occupancy_map:main',
            'filter_saved_map = warehouse_mapping.filter_saved_map:main',
            'pointcloud_to_occupancy_grid = warehouse_mapping.pointcloud_to_occupancy_grid:main',
        ],
    },
)
