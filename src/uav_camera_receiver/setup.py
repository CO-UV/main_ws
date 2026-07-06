import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_camera_receiver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duhun',
    maintainer_email='duhun@todo.todo',
    description='UAV 압축 카메라 토픽을 수신해 raw 이미지로 재발행하는 패키지',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_decompressor_node = uav_camera_receiver.image_decompressor_node:main',
        ],
    },
)
