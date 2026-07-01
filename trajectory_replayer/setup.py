from setuptools import setup
import os
from glob import glob

package_name = 'trajectory_replayer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.mcap')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 node that replays a recorded vehicle trajectory from an mcap bag as an ExternalObjectList',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'trajectory_replayer = trajectory_replayer.trajectory_replayer:main',
        ],
    },
)
