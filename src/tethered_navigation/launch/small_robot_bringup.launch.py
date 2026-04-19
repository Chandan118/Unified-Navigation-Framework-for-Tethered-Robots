from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Bring up the Small Robot Navigator within tethered_navigation.

    Assumes other processes are already providing:
    - /odom
    - /scan
    - /imu/data
    - /ultrasonic_distances
    - camera/image_raw
    """

    navigator_node = Node(
        package='tethered_navigation',
        executable='small_robot_navigator.py',
        name='small_robot_navigator',
        output='screen',
        parameters=[
            {'max_linear_speed': 0.3},
            {'max_angular_speed': 1.0},
        ],
    )

    return LaunchDescription([
        navigator_node,
    ])
