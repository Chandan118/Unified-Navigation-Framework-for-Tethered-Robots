from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Tether Estimator - publishes virtual robot tether lines
    tether_node = Node(
        package='swarm_tether_estimator',
        executable='tether_estimator_node',
        name='tether_estimator',
        parameters=[
            {'world_frame': 'map'},
            {'tether_thickness_m': 0.02},
            {'publish_rate_hz': 20.0},
            {'n_virtual_robots': 3}
        ],
        output='screen'
    )
    ld.add_action(tether_node)

    return ld
