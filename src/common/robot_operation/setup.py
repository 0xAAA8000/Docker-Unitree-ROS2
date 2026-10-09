import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_operation'

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
    description='Nav2 の Twist を Arduino (operator.ino) のシリアル指令に変換する',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'calibrate_drive = robot_operation.calibrate_drive:main',
            'cmd_vel_converter = robot_operation.cmd_vel_converter:main',
        ],
    },
)
