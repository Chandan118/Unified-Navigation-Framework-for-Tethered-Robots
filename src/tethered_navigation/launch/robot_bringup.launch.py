import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('tethered_navigation')

    # --- File Paths ---
    urdf_file_path = os.path.join(pkg_share, 'urdf/my_robot.urdf.xacro')
    ekf_config_path = os.path.join(pkg_share, 'config/ekf.yaml')
    merger_config_path = os.path.join(pkg_share, 'config/laser_merger.yaml')

    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    arduino_port = LaunchConfiguration('arduino_port')
    imu_port = LaunchConfiguration('imu_port')
    front_lidar_port = LaunchConfiguration('front_lidar_port')
    rear_lidar_port = LaunchConfiguration('rear_lidar_port')
    camera_device = LaunchConfiguration('camera_device')
    enable_arduino = LaunchConfiguration('enable_arduino')
    enable_camera_detector = LaunchConfiguration('enable_camera_detector')
    enable_custom_navigator = LaunchConfiguration('enable_custom_navigator')
    enable_camera_publisher = LaunchConfiguration('enable_camera_publisher')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_front_lidar = LaunchConfiguration('enable_front_lidar')
    enable_rear_lidar = LaunchConfiguration('enable_rear_lidar')
    enable_laser_merger = LaunchConfiguration('enable_laser_merger')
    front_lidar_baudrate = LaunchConfiguration('front_lidar_baudrate')
    rear_lidar_baudrate = LaunchConfiguration('rear_lidar_baudrate')
    front_lidar_scan_mode = LaunchConfiguration('front_lidar_scan_mode')
    rear_lidar_scan_mode = LaunchConfiguration('rear_lidar_scan_mode')
    front_lidar_reset_delay_sec = LaunchConfiguration('front_lidar_reset_delay_sec')
    stop_distance_m = LaunchConfiguration('stop_distance_m')
    slow_distance_m = LaunchConfiguration('slow_distance_m')

    camera_pipeline = [
        'v4l2src device=',
        camera_device,
        ' ! video/x-raw, width=1280, height=720, framerate=20/1 ! ',
        'videoconvert ! video/x-raw, format=BGR ! appsink drop=true'
    ]

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyCH341USB0'),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_IG14859004F004A-if00-port0'
        ),
        DeclareLaunchArgument(
            'front_lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        ),
        DeclareLaunchArgument('rear_lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('front_lidar_baudrate', default_value='115200'),
        DeclareLaunchArgument('rear_lidar_baudrate', default_value='115200'),
        DeclareLaunchArgument('front_lidar_scan_mode', default_value='Standard'),
        DeclareLaunchArgument('rear_lidar_scan_mode', default_value='Standard'),
        DeclareLaunchArgument('front_lidar_reset_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('enable_arduino', default_value='true'),
        DeclareLaunchArgument('enable_camera_publisher', default_value='false'),
        DeclareLaunchArgument('enable_camera_detector', default_value='true'),
        DeclareLaunchArgument('enable_custom_navigator', default_value='false'),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('enable_front_lidar', default_value='true'),
        DeclareLaunchArgument('enable_rear_lidar', default_value='false'),
        DeclareLaunchArgument('enable_laser_merger', default_value='false'),
        DeclareLaunchArgument('stop_distance_m', default_value='0.28'),
        DeclareLaunchArgument('slow_distance_m', default_value='0.45'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),
        Node(
            package='tethered_navigation',
            executable='jetson_camera_publisher.py',
            name='jetson_camera_publisher',
            output='screen',
            parameters=[{
                'gstreamer_pipeline': camera_pipeline,
                'camera_device': camera_device,
                'frame_id': 'camera_link'
            }],
            condition=IfCondition(enable_camera_publisher)
        ),
        Node(
            package='tethered_navigation',
            executable='camera_obstacle_detector.py',
            name='camera_obstacle_detector',
            output='screen',
            condition=IfCondition(enable_camera_detector)
        ),
        Node(
            package='tethered_navigation',
            executable='arduino_interface_node.py',
            name='arduino_interface_node',
            output='screen',
            parameters=[{'serial_port': arduino_port}],
            remappings=[('/cmd_vel', '/cmd_vel_safe')],
            condition=IfCondition(enable_arduino)
        ),
        Node(
            package='tethered_navigation',
            executable='safety_cmd_vel_filter.py',
            name='safety_cmd_vel_filter',
            output='screen',
            parameters=[{
                'stop_distance_m': stop_distance_m,
                'slow_distance_m': slow_distance_m,
            }]
        ),
        Node(
            namespace='imu',
            package='lpms_ig1',
            executable='lpms_ig1_node',
            name='lpms_ig1_node',
            output='screen',
            parameters=[{'port': imu_port, 'frame_id': 'imu_link'}],
            condition=IfCondition(enable_imu)
        ),
        Node(
            namespace='imu',
            package='lpms_ig1',
            executable='quat_to_euler_node',
            name='quat_to_euler_node',
            condition=IfCondition(enable_imu)
        ),
        TimerAction(
            period=front_lidar_reset_delay_sec,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'tethered_navigation', 'rplidar_managed_runner.py',
                        '--port', front_lidar_port,
                        '--baud', front_lidar_baudrate,
                        '--frame-id', 'front_lidar_link',
                        '--scan-mode', front_lidar_scan_mode,
                        '--restart-delay', '1.5',
                    ],
                    output='screen',
                ),
            ],
            condition=IfCondition(enable_front_lidar)
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='front_rplidar_node_merged',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'serial_port': front_lidar_port,
                'serial_baudrate': ParameterValue(front_lidar_baudrate, value_type=int),
                'frame_id': 'front_lidar_link',
                'scan_mode': front_lidar_scan_mode,
            }],
            remappings=[('/scan', '/scan_front')],
            condition=IfCondition(enable_laser_merger)
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rear_rplidar_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'serial_port': rear_lidar_port,
                'serial_baudrate': ParameterValue(rear_lidar_baudrate, value_type=int),
                'frame_id': 'rear_lidar_link',
                'scan_mode': rear_lidar_scan_mode,
            }],
            remappings=[('/scan', '/scan_rear')],
            condition=IfCondition(enable_rear_lidar)
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_merger',
            output='screen',
            parameters=[merger_config_path],
            remappings=[
                ('/scan_filtered', '/scan')
            ],
            condition=IfCondition(enable_laser_merger)
        ),
        Node(
            package='tethered_navigation',
            executable='navigator_node',
            name='navigator_node',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_nav')],
            condition=IfCondition(enable_custom_navigator)
        ),
        Node(
            package='tethered_navigation',
            executable='fuzzy_controller.py',
            name='fuzzy_controller',
            output='screen',
            condition=IfCondition(enable_custom_navigator)
        )
    ])