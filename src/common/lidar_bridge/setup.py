import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lidar_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Unitree L1/L2 のトピックを sim と同じ形で Point-LIO と Nav2 に配信する',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_bridge = lidar_bridge.lidar_bridge_node:main',
            'check_inputs = lidar_bridge.check_inputs_node:main',
            'sensor_check = lidar_bridge.sensor_check:main',
            'watch_gaps = lidar_bridge.watch_gaps:main',
        ],
    },
)
