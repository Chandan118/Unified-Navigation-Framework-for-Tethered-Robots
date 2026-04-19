import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tethered_navigation')
    fastdds_profile = os.path.join(pkg_share, 'config', 'fastdds_no_shm.xml')
    nav2_params_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params',
        'nav2_params.yaml'
    )
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_path,
        root_key='',
        param_rewrites={
            'amcl.ros__parameters.base_frame_id': 'base_link',
            # --- Smoother, safer real-world tuning ---
            'controller_server.ros__parameters.controller_frequency': '12.0',
            'controller_server.ros__parameters.min_theta_velocity_threshold': '0.05',
            'controller_server.ros__parameters.failure_tolerance': '0.2',
            'controller_server.ros__parameters.progress_checker.movement_time_allowance': '10.0',
            'controller_server.ros__parameters.FollowPath.max_vel_x': LaunchConfiguration('max_linear_speed'),
            'controller_server.ros__parameters.FollowPath.max_speed_xy': LaunchConfiguration('max_linear_speed'),
            'controller_server.ros__parameters.FollowPath.max_vel_theta': LaunchConfiguration('max_angular_speed'),
            'controller_server.ros__parameters.FollowPath.acc_lim_x': '0.20',
            'controller_server.ros__parameters.FollowPath.acc_lim_theta': '0.70',
            'controller_server.ros__parameters.FollowPath.decel_lim_x': '-0.25',
            'controller_server.ros__parameters.FollowPath.decel_lim_theta': '-0.9',
            'controller_server.ros__parameters.FollowPath.PathAlign.scale': '26.0',
            'controller_server.ros__parameters.FollowPath.PathDist.scale': '34.0',
            'controller_server.ros__parameters.FollowPath.GoalDist.scale': '22.0',
            'controller_server.ros__parameters.FollowPath.RotateToGoal.slowing_factor': '7.0',
            'controller_server.ros__parameters.FollowPath.xy_goal_tolerance': '0.20',
            'controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance': '0.20',
            'controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance': '0.25',
            'local_costmap.local_costmap.ros__parameters.update_frequency': '12.0',
            'local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius': LaunchConfiguration('local_inflation_radius'),
            'local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor': '2.2',
            'global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius': LaunchConfiguration('global_inflation_radius'),
            'global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor': '2.2',
            'behavior_server.ros__parameters.cycle_frequency': '6.0',
        },
        convert_types=True,
    )
    default_map = '/home/jetson/formica_map.yaml'
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    arduino_port = LaunchConfiguration('arduino_port')
    imu_port = LaunchConfiguration('imu_port')
    front_lidar_port = LaunchConfiguration('front_lidar_port')
    rear_lidar_port = LaunchConfiguration('rear_lidar_port')
    front_lidar_baudrate = LaunchConfiguration('front_lidar_baudrate')
    rear_lidar_baudrate = LaunchConfiguration('rear_lidar_baudrate')
    front_lidar_scan_mode = LaunchConfiguration('front_lidar_scan_mode')
    rear_lidar_scan_mode = LaunchConfiguration('rear_lidar_scan_mode')
    front_lidar_reset_delay_sec = LaunchConfiguration('front_lidar_reset_delay_sec')
    camera_device = LaunchConfiguration('camera_device')
    enable_camera_publisher = LaunchConfiguration('enable_camera_publisher')
    enable_camera_detector = LaunchConfiguration('enable_camera_detector')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    local_inflation_radius = LaunchConfiguration('local_inflation_radius')
    global_inflation_radius = LaunchConfiguration('global_inflation_radius')
    stop_distance_m = LaunchConfiguration('stop_distance_m')
    slow_distance_m = LaunchConfiguration('slow_distance_m')
    nav2_start_delay_sec = LaunchConfiguration('nav2_start_delay_sec')
    initial_pose_delay_sec = LaunchConfiguration('initial_pose_delay_sec')
    initial_pose_repeat_count = LaunchConfiguration('initial_pose_repeat_count')
    initial_pose_repeat_interval_sec = LaunchConfiguration('initial_pose_repeat_interval_sec')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')

    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch'),
            '/robot_bringup.launch.py']),
        launch_arguments={
            'enable_custom_navigator': 'false',
            'enable_camera_publisher': enable_camera_publisher,
            'enable_camera_detector': enable_camera_detector,
            'enable_arduino': 'true',
            'arduino_port': arduino_port,
            'imu_port': imu_port,
            'front_lidar_port': front_lidar_port,
            'rear_lidar_port': rear_lidar_port,
            'front_lidar_baudrate': front_lidar_baudrate,
            'rear_lidar_baudrate': rear_lidar_baudrate,
            'front_lidar_scan_mode': front_lidar_scan_mode,
            'rear_lidar_scan_mode': rear_lidar_scan_mode,
            'front_lidar_reset_delay_sec': front_lidar_reset_delay_sec,
            'camera_device': camera_device,
            'stop_distance_m': stop_distance_m,
            'slow_distance_m': slow_distance_m,
        }.items()
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': configured_nav2_params,
            'autostart': 'true',
            'slam': 'False',
            'use_composition': 'False',
            'use_respawn': 'True',
        }.items(),
    )
    delayed_nav2_bringup = TimerAction(period=nav2_start_delay_sec, actions=[nav2_bringup_launch])

    initial_pose_node = Node(
        package='tethered_navigation',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'x': initial_pose_x,
            'y': initial_pose_y,
            'yaw': initial_pose_yaw,
            # Keep publishing long enough for AMCL to be active on slower bringups.
            'delay_sec': initial_pose_delay_sec,
            'repeat_count': initial_pose_repeat_count,
            'repeat_interval_sec': initial_pose_repeat_interval_sec,
            'frame_id': 'map',
        }],
        condition=IfCondition(publish_initial_pose),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
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
        DeclareLaunchArgument('enable_camera_publisher', default_value='false'),
        DeclareLaunchArgument('enable_camera_detector', default_value='false'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.14'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.65'),
        DeclareLaunchArgument('local_inflation_radius', default_value='0.60'),
        DeclareLaunchArgument('global_inflation_radius', default_value='0.70'),
        DeclareLaunchArgument('stop_distance_m', default_value='0.28'),
        DeclareLaunchArgument('slow_distance_m', default_value='0.45'),
        DeclareLaunchArgument('nav2_start_delay_sec', default_value='8.0'),
        DeclareLaunchArgument('initial_pose_delay_sec', default_value='9.0'),
        DeclareLaunchArgument('initial_pose_repeat_count', default_value='10'),
        DeclareLaunchArgument('initial_pose_repeat_interval_sec', default_value='2.0'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('publish_initial_pose', default_value='true'),
        # Force all ROS discovery/traffic local to this Jetson for reliable service discovery.
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        # Force FastDDS shared memory transport fully off (Jetson SHM lock issues).
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        robot_bringup_launch,
        delayed_nav2_bringup,
        initial_pose_node,
    ])