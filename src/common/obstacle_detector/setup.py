from setuptools import find_packages, setup

package_name = 'obstacle_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='LiDAR の点群から Nav2 の障害物 (坂を除く) とクリア用の点群を作る',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'obstacle_detector = obstacle_detector.obstacle_detector:main',
        ],
    },
)
