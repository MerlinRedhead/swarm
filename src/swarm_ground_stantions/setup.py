from setuptools import setup
import os
from glob import glob

package_name = 'swarm_ground_stantions'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='merlin',
    description='Ground Station',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gui_node = swarm_ground_stantions.gui_node:main',
            'yolo_processor = swarm_ground_stantions.yolo_processor:main',
        ],
    },
)
