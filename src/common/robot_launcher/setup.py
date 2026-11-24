import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='スマホからモード (目的地設定の自走, キャリブレーション) を選んで起動する',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'launcher = robot_launcher.launcher:main',
            'calib_web = robot_launcher.calib_web:main',
        ],
    },
)
