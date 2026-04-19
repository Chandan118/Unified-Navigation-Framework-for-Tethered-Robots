from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Aliengo Bridge - connects swarm tether states to Unitree robot
    bridge_node = Node(
        package='unitree_aliengo_bridge',
        executable='aliengo_bridge_node',
        name='aliengo_bridge',
        parameters=[
            {'max_speed_mps': 0.4},
            {'safety_margin_m': 0.5},
            {'yield_stop_duration_s': 2.0}
        ],
        output='screen'
    )
    ld.add_action(bridge_node)

    return ld
