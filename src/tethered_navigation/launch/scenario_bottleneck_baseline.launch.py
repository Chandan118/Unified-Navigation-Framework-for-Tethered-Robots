"""
Swarm Bottleneck Scenario - Baseline (Independent Robots)
==========================================================
场景 A: 狭窄走廊 - Baseline 模式
机器人独立运行，不进行 swarm 通信
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'tethered_navigation')
    
    # 使用通用实验 launcher，指定场景和模式
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'swarm_experiment.launch.py')
        ),
        launch_arguments={
            'scenario': 'bottleneck',
            'mode': 'baseline',
            'num_robots': '5',
            'use_sim_time': 'true',
        }.items()
    )
    
    return LaunchDescription([
        LogInfo(msg="=" * 60),
        LogInfo(msg="Swarm Experiment: Bottleneck - BASELINE"),
        LogInfo(msg="Description: 5 robots navigate through narrow corridor independently"),
        LogInfo(msg="Expected: Higher entanglement rate due to no coordination"),
        LogInfo(msg="=" * 60),
        main_launch
    ])
