import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('atlas_gazebo')
    pkg_description = get_package_share_directory('atlas_description')
    
    # Path to the Xacro file
    xacro_file = os.path.join(pkg_description, 'urdf', 'atlas_t.xacro')
    
    # Robot description using Xacro
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_count = 10
    robot_prefix = 'robot_'
    
    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_gazebo, 'worlds', 'warehouse_world.world')}.items()
    )
    
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo
    ])
    
    # Spawn swarm
    for i in range(1, robot_count + 1):
        robot_name = f"{robot_prefix}{i}"
        
        # Simple ring distribution for spawning
        import math
        radius = 5.0
        angle = 2.0 * math.pi * (i-1) / robot_count
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        robot_group = GroupAction([
            PushRosNamespace(robot_name),
            
            # Robot State Publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description_config,
                    'frame_prefix': f"{robot_name}/"
                }]
            ),
            
            # Spawn Entity
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description', 
                    '-entity', robot_name, 
                    '-x', str(x), '-y', str(y), '-z', '0.1'
                ],
                output='screen'
            )
        ])
        
        ld.add_action(robot_group)
        
    return ld
