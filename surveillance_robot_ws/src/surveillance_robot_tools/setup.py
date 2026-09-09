from setuptools import find_packages, setup

package_name = 'surveillance_robot_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Development and diagnostic utilities for the surveillance robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Utility nodes are registered here in a later chunk, e.g.:
            # 'diagnostic_node = surveillance_robot_tools.diagnostic_node:main',
            # 'telemetry_node = surveillance_robot_tools.telemetry_node:main',
        ],
    },
)
