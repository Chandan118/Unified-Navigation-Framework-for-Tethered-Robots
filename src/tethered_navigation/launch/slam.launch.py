import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot hardware drivers
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('tethered_navigation'), 'launch'),
            '/robot_bringup.launch.py'])
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory("slam_toolbox") + '/config/mapper_params_online_async.yaml',
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        robot_bringup_launch,
        slam_toolbox_node
    ])