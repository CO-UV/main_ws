import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ugv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models', 'ugv_diff'),
            glob('models/ugv_diff/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autonav009',
    maintainer_email='jskim010903@inha.edu',
    description='Differential-drive UGV spawn and pure-pursuit path following for the warehouse.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pure_pursuit = ugv_bringup.pure_pursuit:main',
        ],
    },
)
