"""
Swarm Crossing Paths Scenario - Cooperative
============================================
场景 B: 交叉路径 - Cooperative 模式
4-8 机器人从对侧出发，tether 必然交叉
测试 cooperative 机制的有效性
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
            'scenario': 'crossing',
            'mode': 'cooperative',
            'num_robots': '6',
            'use_sim_time': 'true',
        }.items()
    )
    
    return LaunchDescription([
        LogInfo(msg="=" * 60),
        LogInfo(msg="Swarm Experiment: Crossing Paths - COOPERATIVE"),
        LogInfo(msg="Description: 6 robots cross in X-pattern with central obstacle"),
        LogInfo(msg="Expected: Clear yielding/reroute behaviors observed"),
        LogInfo(msg="=" * 60),
        main_launch
    ])
