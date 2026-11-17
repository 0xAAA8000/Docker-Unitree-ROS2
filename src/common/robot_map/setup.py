import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Point-LIO の 3D 点群から 2D 地図 (/map) を作って配信する',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'map = robot_map.map_node:main',
        ],
    },
)
