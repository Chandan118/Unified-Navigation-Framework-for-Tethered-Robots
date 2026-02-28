import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('hybrid_navigation')
    
    # 1. Start Hybrid Navigation Stack (included)
    # We pass rviz=false and use_sim_time=false as per the original ROS 1 file
    hybrid_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hybrid_navigation.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'use_sim_time': 'false'
        }.items()
    )
    
    return LaunchDescription([
        # 2. Mock Data Generator
        Node(
            package='hybrid_navigation',
            executable='mock_data_generator',
            name='mock_data_generator',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # Included Launch
        hybrid_nav_launch,
        
        # 3. Data Logger
        Node(
            package='hybrid_navigation',
            executable='data_logger',
            name='data_logger',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'log_dir': os.path.join(pkg_share, '..', '..', 'results')
            }]
        )
    ])
