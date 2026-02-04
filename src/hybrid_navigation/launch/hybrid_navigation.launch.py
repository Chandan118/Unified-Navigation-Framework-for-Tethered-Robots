import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('hybrid_navigation')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
        
        # Sensor Fusion Node
        Node(
            package='hybrid_navigation',
            executable='sensor_fusion',
            name='sensor_fusion_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Scene Recognition Node
        Node(
            package='hybrid_navigation',
            executable='scene_recognition',
            name='scene_recognition_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Tether Tension Node
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
        
        # Hybrid Planner Node
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
        
        # Fuzzy Logic Controller
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
        
        # Genetic Algorithm Optimizer
        Node(
            package='hybrid_navigation',
            executable='ga_optimizer',
            name='ga_optimizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'population_size': 10,
                'mutation_rate': 0.1,
                'crossover_rate': 0.7,
                'optimization_interval': 120.0
            }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=LaunchConfiguration('rviz')
        )
    ])
