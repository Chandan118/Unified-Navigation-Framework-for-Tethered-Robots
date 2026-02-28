import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('hybrid_navigation')
    try:
        gazebo_pkg_share = get_package_share_directory('atlas_gazebo')
    except:
        gazebo_pkg_share = None
        
    # 1. Spawn swarm of robots in Gazebo (included)
    spawn_swarm_launch = None
    if gazebo_pkg_share:
        spawn_swarm_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg_share, 'launch', 'spawn_swarm.launch.py')
            )
        )
        
    # 2. Start swarm hybrid navigation stack (included)
    swarm_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hybrid_navigation_swarm.launch.py')
        )
    )
    
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ])
    
    if spawn_swarm_launch:
        ld.add_action(spawn_swarm_launch)
        
    ld.add_action(swarm_nav_launch)
    
    return ld
