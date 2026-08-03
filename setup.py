# Copyright 2023-2026 Hacklab Kuopio
#
# Licensed under the Apache License, Version 2.0.
# See the LICENSE file in the project root for license information.

# encoding: utf-8
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'litter_collector_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        ('lib/' + package_name, [package_name+'/roboclaw_3.py']),
        ('lib/' + package_name, [package_name+'/roboclaw_error_logger.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lamikae',
    maintainer_email='mika+lcr@lamikae.net',
    description='Litter Collector Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_node = litter_collector_robot.robot_control_node:main',
            'brake_control_node = litter_collector_robot.brake_control_node:main',
            'roboclaw_twist_node = litter_collector_robot.roboclaw_twist_node:main',
            'nose_node = litter_collector_robot.nose_node:main',
            'vacuum_node = litter_collector_robot.vacuum_node:main',
            'health_node = litter_collector_robot.health_node:main',
            'lights_node = litter_collector_robot.lights_node:main',
            'audio_node = litter_collector_robot.audio_node:main',
            'emergency_stop_node = litter_collector_robot.emergency_stop_node:main',
            'shutdown_node = litter_collector_robot.shutdown_node:main',
        ],
    },
)
