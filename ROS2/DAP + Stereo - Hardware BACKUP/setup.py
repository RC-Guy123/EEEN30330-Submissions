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
         # Install the entire config directory
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
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
            #'daf_avoidance_node = my_package.daf_avoidance_node:main',
            'stereo_obstacle_processor = my_package.stereo_obstacle_processor:main',
            'rpicam_stereo_node = my_package.rpicam_stereo_node:main',
            'stereo_capture = my_package.stereo_capture:main',
            'pointcloud_filter = my_package.pointcloud_filter:main',
            'load_stereo_calibration = my_package.load_stereo_calibration:main',
        ],
    },
)