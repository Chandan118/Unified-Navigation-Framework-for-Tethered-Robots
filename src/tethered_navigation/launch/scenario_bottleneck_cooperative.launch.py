"""
Swarm Bottleneck Scenario - Cooperative (With Swarm Coordination)
==================================================================
场景 A: 狭窄走廊 - Cooperative 模式
机器人使用 swarm_tether_states 进行协调
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'tethered_navigation')
    
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'swarm_experiment.launch.py')
        ),
        launch_arguments={
            'scenario': 'bottleneck',
            'mode': 'cooperative',
            'num_robots': '5',
            'use_sim_time': 'true',
        }.items()
    )
    
    return LaunchDescription([
        LogInfo(msg="=" * 60),
        LogInfo(msg="Swarm Experiment: Bottleneck - COOPERATIVE"),
        LogInfo(msg="Description: 5 robots coordinate via swarm_tether_states"),
        LogInfo(msg="Expected: Lower entanglement, yield/reroute behaviors emerge"),
        LogInfo(msg="=" * 60),
        main_launch
    ])
