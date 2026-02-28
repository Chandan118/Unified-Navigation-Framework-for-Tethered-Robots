import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_share = get_package_share_directory('hybrid_navigation')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_flag = LaunchConfiguration('rviz', default='true')
    rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    
    # Global parameters
    robot_count = 10
    robot_prefix = 'robot_'
    
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ])
    
    # Per-robot navigation stacks
    for i in range(1, robot_count + 1):
        robot_name = f"{robot_prefix}{i}"
        
        robot_group = GroupAction([
            PushRosNamespace(robot_name),
            
            Node(
                package='hybrid_navigation',
                executable='sensor_fusion',
                name='sensor_fusion_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            
            Node(
                package='hybrid_navigation',
                executable='scene_recognition',
                name='scene_recognition_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),
            
            Node(
                package='hybrid_navigation',
                executable='tether_tension',
                name='tether_tension_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'max_tether_length': 30.0,
                    'tension_coefficient': 1.5
                }]
            ),
            
            Node(
                package='hybrid_navigation',
                executable='hybrid_planner',
                name='hybrid_planner_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'obstacle_threshold': 1.5,
                    'goal_tolerance': 0.5,
                    'wall_follow_distance': 0.7,
                    'max_tether_usage': 0.95
                }]
            ),
            
            Node(
                package='hybrid_navigation',
                executable='fuzzy_controller',
                name='fuzzy_controller_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'linear_speed': 0.5,
                    'max_angular_speed': 1.0,
                    'safety_distance': 1.0
                }]
            ),
        ])
        
        ld.add_action(robot_group)
        
    # Swarm coordinator and logger at global level
    ld.add_action(Node(
        package='hybrid_navigation',
        executable='swarm_coordinator',
        name='swarm_coordinator',
        output='screen',
        parameters=[{
            'robot_count': robot_count,
            'robot_prefix': robot_prefix,
            'use_sim_time': use_sim_time
        }]
    ))
    
    ld.add_action(Node(
        package='hybrid_navigation',
        executable='swarm_data_logger',
        name='swarm_data_logger',
        output='screen',
        parameters=[{
            'log_dir': os.path.join(pkg_share, '..', '..', 'results'),
            'robot_count': robot_count,
            'robot_prefix': robot_prefix,
            'use_sim_time': use_sim_time
        }]
    ))
    
    # RViz (global)
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfiguration('rviz')
    ))
    
    return ld
