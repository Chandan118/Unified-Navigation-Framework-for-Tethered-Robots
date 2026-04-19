"""
Swarm Expansion Scenario - Cooperative
========================================
场景 C: Swarm Expansion - Cooperative 模式
10 机器人从中心锚点向外扩展进入密集障碍物区
测试系统可扩展性和最大张力分布
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'tethered_navigation')
    
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'swarm_experiment.launch.py')
        ),
        launch_arguments={
            'scenario': 'expansion',
            'mode': 'cooperative',
            'num_robots': '10',
            'use_sim_time': 'true',
        }.items()
    )
    
    return LaunchDescription([
        LogInfo(msg="=" * 60),
        LogInfo(msg="Swarm Experiment: Expansion - COOPERATIVE (10 robots)"),
        LogInfo(msg="Description: 10 robots deploy from central anchor with 30+ obstacles"),
        LogInfo(msg="Expected: Tests scalability and tension distribution"),
        LogInfo(msg="=" * 60),
        main_launch
    ])
