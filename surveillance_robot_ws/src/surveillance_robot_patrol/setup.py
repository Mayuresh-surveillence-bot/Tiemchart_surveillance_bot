import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'surveillance_robot_patrol'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Custom surveillance patrol behavior for the surveillance robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Patrol nodes are registered here in a later chunk, e.g.:
            # 'patrol_manager = surveillance_robot_patrol.patrol_manager:main',
            # 'mission_manager = surveillance_robot_patrol.mission_manager:main',
        ],
    },
)
