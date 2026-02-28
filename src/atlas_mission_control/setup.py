from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'atlas_mission_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chandan Sheikder',
    maintainer_email='chandan@bit.edu.cn',
    description='The atlas_mission_control package provides high-level mission management for tethered robots.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = atlas_mission_control.nodes.mission_manager_node:main',
        ],
    },
)
