from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Complete mixed-reality swarm experiment: virtual robots + physical Unitree"""
    ld = LaunchDescription()

    # 1. Tether Estimator (virtual robots' tethers)
    tether_node = Node(
        package='swarm_tether_estimator',
        executable='tether_estimator_node',
        name='tether_estimator',
        parameters=[
            {'world_frame': 'map'},
            {'tether_thickness_m': 0.02},
            {'publish_rate_hz': 20.0},
            {'n_virtual_robots': 2}
        ],
        output='screen'
    )
    ld.add_action(tether_node)

    # 2. Unitree Bridge (yielding behavior on physical robot)
    bridge_node = Node(
        package='unitree_aliengo_bridge',
        executable='aliengo_bridge_node',
        name='aliengo_bridge',
        parameters=[
            {'max_speed_mps': 0.4},
            {'safety_margin_m': 0.6},
            {'yield_stop_duration_s': 2.5}
        ],
        output='screen'
    )
    ld.add_action(bridge_node)

    return ld
