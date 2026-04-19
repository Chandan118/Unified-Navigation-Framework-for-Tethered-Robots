"""
Swarm Robotics Experiment Launcher
===================================
主启动文件：启动多机器人 swarm 实验

支持两种模式：
- baseline: 机器人独立运行，无 swarm 通信
- cooperative: 启用 swarm coordination 和 yielding

使用方法:
  ros2 launch tethered_navigation swarm_bottleneck_baseline.launch.py
  ros2 launch tethered_navigation swarm_crossing_cooperative.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
    TimerAction, LogInfo, GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ==================== Launch Arguments ====================
    pkg_share = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'tethered_navigation'
    )
    
    # 场景和模式
    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='bottleneck',
        description='Scenario: bottleneck, crossing, expansion'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='cooperative',
        description='Experiment mode: baseline or cooperative'
    )
    
    num_robots_arg = DeclareLaunchArgument(
        'num_robots', default_value='5',
        description='Number of robots in the swarm'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    
    # ==================== Scenario Configuration ====================
    scenario = LaunchConfiguration('scenario')
    mode = LaunchConfiguration('mode')
    num_robots = LaunchConfiguration('num_robots')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ==================== World File Selection ====================
    world_files = {
        'bottleneck': os.path.join(pkg_share, 'worlds', 'swarm_bottleneck.sdf'),
        'crossing': os.path.join(pkg_share, 'worlds', 'swarm_crossing_paths.sdf'),
        'expansion': os.path.join(pkg_share, 'worlds', 'swarm_expansion.sdf'),
    }
    
    selected_world = PythonExpression({
        'bottleneck': world_files['bottleneck'],
        'crossing': world_files['crossing'],
        'expansion': world_files['expansion'],
    }, scenario)
    
    # ==================== Start Simulation (2D Simulator) ====================
    simulator_node = Node(
        package='tethered_navigation',
        executable='swarm_simulator.py',
        name='swarm_simulator',
        output='screen',
        arguments=[scenario, num_robots],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # ==================== Data Logger ====================
    data_logger = Node(
        package='tethered_navigation',
        executable='experiment_data_logger.py',
        name='experiment_logger',
        output='screen',
        arguments=[scenario, mode, num_robots],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # ==================== Robot Nodes ====================
    # 为每个机器人创建 navigator, fuzzy controller, swarm coordinator (cooperative模式)
    robot_nodes = []
    
    # 导入机器人配置生成函数（简化版）
    def get_robot_config(scenario_name: str, robot_index: int, total_robots: int):
        """返回机器人的起始位置和目标"""
        import random, math
        random.seed(42 + robot_index)
        
        if scenario_name == 'bottleneck':
            # 左侧入口区域 -> 右侧出口区域
            sx = -7.0 + random.uniform(-1.0, 1.0)
            sy = random.uniform(-1.0, 1.0)
            syaw = random.uniform(-0.3, 0.3)
            gx = 7.0 + random.uniform(-1.0, 1.0)
            gy = random.uniform(-1.0, 1.0)
            
        elif scenario_name == 'crossing':
            # 四角交叉
            angles = [0, math.pi/2, math.pi, 3*math.pi/2]
            angle = angles[robot_index % 4]
            radius = 9.0
            sx = radius * math.cos(angle)
            sy = radius * math.sin(angle)
            syaw = angle + math.pi
            gx = -sx
            gy = -sy
            
        elif scenario_name == 'expansion':
            # 从中心向四周扩展
            angle = (2 * math.pi / total_robots) * robot_index
            sx = 0.5 * math.cos(angle)
            sy = 0.5 * math.sin(angle)
            syaw = angle
            goal_radius = random.uniform(8, 12)
            gx = goal_radius * math.cos(angle)
            gy = goal_radius * math.sin(angle)
            
        return (sx, sy, syaw), (gx, gy)
    
    for i in range(int(num_robots)):
        robot_id = f"robot_{i+1}"
        namespace = f"/{robot_id}"
        
        # 获取该机器人的起始和目标位置
        (sx, sy, syaw), (gx, gy) = get_robot_config(scenario, i, int(num_robots))
        
        # Navigator (C++)
        navigator = Node(
            package='tethered_navigation',
            executable='navigator_node',
            name='navigator',
            namespace=namespace,
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'initial_pose_x': sx},
                {'initial_pose_y': sy},
                {'initial_pose_yaw': syaw},
                {'robot_id': robot_id},
                {'max_tether_length': 10.0 if scenario == 'bottleneck' else 15.0},
            ]
        )
        
        # Fuzzy Controller (Python)
        fuzzy_controller = Node(
            package='tethered_navigation',
            executable='fuzzy_controller.py',
            name='fuzzy_controller',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
        
        # Swarm Coordinator (仅 cooperative 模式)
        swarm_nodes = []
        if mode == 'cooperative':
            coordinator = Node(
                package='tethered_navigation',
                executable='swarm_coordinator.py',
                name='swarm_coordinator',
                namespace=namespace,
                output='screen',
                arguments=[robot_id, namespace],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'static_priority': (i + 1) / float(int(num_robots))},  # 基于索引分配优先级
                ]
            )
            
            broadcaster = Node(
                package='tethered_navigation',
                executable='tether_state_broadcaster.py',
                name='tether_broadcaster',
                namespace=namespace,
                output='screen',
                arguments=[robot_id, namespace],
                parameters=[{'use_sim_time': use_sim_time}],
            )
            
            swarm_nodes = [coordinator, broadcaster]
        
        robot_nodes.extend([navigator, fuzzy_controller] + swarm_nodes)
    
    # ==================== Launch Sequence ====================
    # 1. 启动仿真器
    # 2. 延迟启动机器人（等待仿真器就绪）
    # 3. 启动数据记录器
    # 4. 最后启动各个机器人
    
    delayed_simulator = TimerAction(
        period=1.0,
        actions=[simulator_node]
    )
    
    delayed_logger = TimerAction(
        period=2.0,
        actions=[data_logger]
    )
    
    # 机器人节点分批启动，避免同时爆发
    robot_groups = []
    for i, node in enumerate(robot_nodes):
        delay = 3.0 + (i * 0.2)  # 每个间隔 0.2 秒
        robot_groups.append(TimerAction(period=delay, actions=[node]))
    
    return LaunchDescription([
        # 参数声明
        scenario_arg,
        mode_arg,
        num_robots_arg,
        use_sim_time_arg,
        
        # 启动序列
        LogInfo(msg="=" * 60),
        LogInfo(msg=f"启动 Swarm 实验: 场景={scenario}, 模式={mode}, 机器人={num_robots}"),
        LogInfo(msg="=" * 60),
        
        delayed_simulator,
        delayed_logger,
        
        # 启动所有机器人节点
        *robot_groups,
    ])
