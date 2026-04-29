import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),

        # Only copy actual world files, not directories
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),

        # Goal marker model
        (os.path.join('share', package_name, 'models', 'goal_marker'),
         glob('models/goal_marker/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'daf_avoidance_node = my_package.daf_avoidance_node:main',
            'lidar_obstacle_processor = my_package.lidar_obstacle_processor:main',
            'path_logger = my_package.path_logger:main',
        ],
    },
)