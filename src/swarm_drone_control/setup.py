from setuptools import setup
import os
from glob import glob

package_name = 'swarm_drone_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*config.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'hub_relay = swarm_drone_control.hub_relay:main',
            'vision_node = swarm_drone_control.vision_node:main',
            'striker_node = swarm_drone_control.striker_node:main',
            'connect_node = swarm_drone_control.connect_node:main'
        ],
    },
)
