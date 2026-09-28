import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sim_gazebo'

data_files_list = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'world'), glob('world/*.world'))
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files_list,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='okamoto',
    maintainer_email='kuwalin@outlook.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
