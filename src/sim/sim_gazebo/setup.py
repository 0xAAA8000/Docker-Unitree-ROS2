import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sim_gazebo'

py_modules = [ # site-packagesに追加するファイル
    'add_time_field',
]

data_files_list = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    (os.path.join('share', package_name, 'world'), glob('world/*.world')),
    # (インストール先, インストール元)
    # launch.pyのNode関数は実行ファイルを探す。実行ファイルはlibにまとめられる
    #(os.path.join('lib', package_name), glob('add_time_field.py'))
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=py_modules,
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
            # launchで指定する実行ファイル名 = package名.script名:main関数名
            'add_time_field = add_time_field:main',
        ],
    },
)
