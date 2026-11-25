import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
        (os.path.join('share', package_name, 'web', 'vendor'), glob('web/vendor/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='コマンドラインやスマホのブラウザから Nav2 に目的地を送る',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'goal = robot_goal.goal:main',
            'web = robot_goal.web:main',
        ],
    },
)
