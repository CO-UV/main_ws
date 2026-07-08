from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_mapping'

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
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
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
        ],
    },
)
