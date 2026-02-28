from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hybrid_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chandan Sheikder',
    maintainer_email='chandan@bit.edu.cn',
    description='Hybrid navigation stack with fuzzy logic and tether awareness for ATLAS-T',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fuzzy_controller = hybrid_navigation.nodes.fuzzy_controller_node:main',
            'hybrid_planner = hybrid_navigation.nodes.hybrid_planner_node:main',
            'tether_tension = hybrid_navigation.nodes.tether_tension_node:main',
            'scene_recognition = hybrid_navigation.nodes.scene_recognition_node:main',
            'sensor_fusion = hybrid_navigation.nodes.sensor_fusion_node:main',
            'ga_optimizer = hybrid_navigation.nodes.ga_optimizer_node:main',
            'data_logger = hybrid_navigation.nodes.data_logger_node:main',
            'mock_data_generator = hybrid_navigation.nodes.mock_data_generator:main',
            'swarm_coordinator = hybrid_navigation.nodes.swarm_coordinator_node:main',
            'swarm_data_logger = hybrid_navigation.nodes.swarm_data_logger_node:main',
        ],
    },
)
