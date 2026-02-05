import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('hybrid_navigation')
    # Assuming atlas_gazebo is also migrated or follows same structure
    try:
        gazebo_pkg_share = get_package_share_directory('atlas_gazebo')
    except:
        gazebo_pkg_share = None
    
    # 1. Spawn Robot in Gazebo (included)
    spawn_robot_launch = None
    if gazebo_pkg_share:
        spawn_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg_share, 'launch', 'spawn_robot.launch.py')
            )
        )
    
    # 2. Start Hybrid Navigation Stack (included)
    hybrid_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hybrid_navigation.launch.py')
        )
    )
    
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ])
    
    if spawn_robot_launch:
        ld.add_action(spawn_robot_launch)
    
    ld.add_action(hybrid_nav_launch)
    
    # 3. Data Logger
    ld.add_action(Node(
        package='hybrid_navigation',
        executable='data_logger',
        name='data_logger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'log_dir': os.path.join(pkg_share, '..', '..', 'results')
        }]
    ))
    
    # 4. RQT Plot (optional, ROS 2 version is rqt_plot as well)
    # Node(package='rqt_plot', executable='rqt_plot', ...) 
    
    return ld
