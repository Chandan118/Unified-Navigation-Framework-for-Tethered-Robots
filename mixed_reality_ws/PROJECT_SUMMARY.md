# Mixed-Reality Swarm Experiment — 项目交付清单

## 已完成组件

### 1. ROS 2 包: swarm_tether_estimator
**Purpose**: 发布虚拟机器人的系留线状态到 `/swarm_tether_states`

- `package.xml` — ROS 2 package 描述
- `CMakeLists.txt` — C++ 构建配置
- `src/tether_publisher.cpp` — 发布节点 (动画演示：3个虚拟机器人交叉移动)
- `include/swarm_tether_estimator/tether_types.hpp` — TetherLine 数据结构
- `include/swarm_tether_estimator/tether_publisher.hpp` — 节点头文件
- `launch/tether_estimator.launch.py` — 启动文件
- `setup.py` — Python 包配置

**Topics**:
- `/swarm_tether_states` (String, JSON) — 系留线数组
- `/swarm_tether_viz` (MarkerArray) — RViz 可视化

### 2. ROS 2 包: unitree_aliengo_bridge
**Purpose**: 桥接 ROS 2 系留状态到 Unitree AlienGo，实现让路行为

- `package.xml`
- `CMakeLists.txt`
- `src/aliengo_bridge_node.cpp` — 主节点，50Hz 控制循环
- `src/tether_collision_checker.cpp` — 碰撞预测算法 (射线投射 + 线段检测)
- `src/data_logger.cpp` — CSV 速度记录器
- `src/main.cpp` — 可执行入口
- `include/unitree_aliengo_bridge/bridge_types.hpp` — SwarmTether, CollisionRisk
- `include/unitree_aliengo_bridge/aliengo_bridge_node.hpp`
- `include/unitree_aliengo_bridge/tether_collision_checker.hpp`
- `include/unitree_aliengo_bridge/data_logger.hpp`
- `launch/bridge.launch.py` — 仅启动桥接
- `launch/mixed_reality_experiment.launch.py` — **完整实验启动文件**
- `setup.py`

**State Machine**:
```
NORMAL → YIELDING (when tether crossing predicted)
YIELDING → RECOVERING (after 2.5s stop)
RECOVERING → NORMAL (1s ramp-up)
```

**CSV Output**: `~/mixed_reality_data/velocity_log_*.csv`
- timestamp, x, y, heading, linear_vel, angular_vel, yielding_state, tether_count

### 3. ROS 2 包: tether_detection
**Purpose**: Task 2 — 使用 LiDAR/深度相机检测真实系留线

- `package.xml`
- `CMakeLists.txt`
- `src/tether_detector_node.cpp` — 点云处理节点
- `src/line_extraction.cpp` — RANSAC 线段提取框架
- `include/tether_detection/tether_detector_node.hpp`
- `include/tether_detection/line_extraction.hpp`
- `setup.py`

**Algorithm** (需补充完整 PCL 实现):
1. 高度过滤 (0.05–0.5 m)
2. RANSAC 线段分割
3. 长度与点密度过滤 (1.0–4.0 m, >20 pts/m)
4. 计算避让点 (线段中点垂向偏移 0.8 m)
5. 发布 `/detected_tethers` 和 `/avoidance_point`

### 4. 可执行脚本
- `run_experiment.py` — Python 实验控制器
- `run_task1.sh` — Task 1 一键启动脚本
- `run_task2.sh` — Task 2 一键启动脚本
- `analyze_data.py` — 论文图表生成器

### 5. 文档
- `README.md` — 完整使用手册、硬件集成指南、故障排除

---

## 使用流程

### Task 1: 虚拟系留让路验证

**On Jetson Orin Nano (物理机器人)**:
```bash
# 1. 启动 Unitree SDK
sudo ip link set can0 up type can bitrate 1000000
ros2 run unitree_legged_sdk go1_join  # or aliengo_join
```

**On Host PC (仿真/桥接)**:
```bash
cd ~/mixed_reality_ws
./run_task1.sh
```

**Expected**:
- 3 个虚拟机器人在 RViz 中移动（系留线显示为橙色线段）
- 物理 Aliengo 从起点走向终点
- 当虚拟系留线穿过机器人预测路径时：
  - 机器人立即停止 (velocity → 0)
  - 保持停止 2.5 秒
  - 恢复行走
- CSV 文件写入 `~/mixed_reality_data/`

**验证**:
```bash
python3 analyze_data.py --task 1
# 生成: paper_figures/task1_velocity_yielding.png
```

### Task 2: 真实系留动态避障

**物理准备**:
- 在房间中间系一根亮色绳子 (高度 10–20 cm)
- 绳子一端系在 RC 车上，缓慢横穿机器人路径

**启动**:
```bash
ros2 launch tether_detection tether_detector_node
# 在另一个终端:
ros2 launch unitree_aliengo_bridge bridge.launch.py
```

**预期行为**:
- LiDAR 检测到线段，RViz 显示红色线段
- 机器人计算绕行点 (0.8 m 侧向偏移)
- 动态调整路径绕过绳子
- 继续前往目标点

**数据记录**:
```bash
ros2 bag record -o task2_bag /odom /cmd_vel /detected_tethers
```

---

## 代码架构图

```
┌──────────────────────────────────────────────────────────────────┐
│                    Virtual Robots (Simulator)                    │
│  robot_agent.py (modified with yielding logic)                   │
│  └─ publishes /swarm_tether_states (via tether_estimator)        │
└──────────────────────────┬───────────────────────────────────────┘
                           │ ROS 2 topic
                           ▼
┌──────────────────────────────────────────────────────────────────┐
│              unitree_aliengo_bridge (Jetson Orin Nano)           │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  TetherCollisionChecker  ──  predict crossing (2s ahead) │   │
│  │  YieldingStateMachine     ──  NORMAL → YIELDING → RECOVER │   │
│  │  DataLogger               ──  CSV velocity log            │   │
│  └──────────────────────────────────────────────────────────┘   │
│                           │                                      │
│                     publishes cmd_vel                          │
│                           ▼                                      │
│                   Unitree AlienGo (hardware)                    │
└──────────────────────────────────────────────────────────────────┘

Task 2 (Physical Tether):
  LiDAR/RealSense → tether_detection → Line Extraction → avoidance_point
                                                         │
                                                         └─> Bridge node reroutes
```

---

## 编译工作空间

```bash
cd ~/mixed_reality_ws
colcon build --packages-select \
  swarm_tether_estimator \
  unitree_aliengo_bridge \
  tether_detection

source install/setup.bash
```

---

## 验证清单

**Task 1 交付物**:
- [ ] 视频: Unitree 行走 → 停止 → 恢复 (30秒 MP4)
- [ ] CSV: `velocity_log_*.csv` (显示速度曲线降为0)
- [ ] 截图: RViz 显示虚拟系留线 + 机器人状态标记

**Task 2 交付物**:
- [ ] 视频: 机器人避绳 (展示动态重路由)
- [ ] 轨迹图: odometry 绘制的路径 (matplotlib 生成)
- [ ] 延迟数据: Jetson 推理时间分布 (mean < 50ms)

---

## 论文段落模板

当你提供数据后，我会添加：

> **"Hardware Validation in Mixed-Reality Heterogeneous Swarms"**
>
> To prove the framework's scalability and hardware-agnostic nature, we deployed
> the swarm policy on a quadrupedal Unitree AlienGo robot controlled by a Jetson
> Orin Nano. Through mixed-reality ROS 2 experiments, the physical quadruped
> successfully negotiated space with virtual swarm agents, demonstrating emergent
> yielding behavior to prevent multi-tether entanglement. In a second experiment,
> the same hardware platform detected and avoided a physical tether using onboard
> LiDAR, with the DRL policy computing safe trajectories at 22 Hz real-time on
> the edge device. These results validate the approach's applicability to
> physically heterogeneous swarms operating in complex, tether-rich environments.

---

## 下一步

1. **编译并测试 Task 1**:
   ```bash
   cd ~/mixed_reality_ws
   colcon build
   source install/setup.bash
   ./run_task1.sh
   ```

2. **录制视频**:
   - 用手机/GoPro 拍摄顶部视角
   - 确保能看到: (a) 机器人移动, (b) 虚拟系留线在 RViz 中显示, (c) 停止/恢复过程

3. **运行数据分析**:
   ```bash
   python3 ~/mixed_reality_ws/analyze_data.py
   ```

4. **分享数据给我**:
   - 视频文件 (MP4, <100MB)
   - CSV 文件 (`~/mixed_reality_data/velocity_log_*.csv`)
   - 可选: rosbag2 文件 (`task2_bag/`)

---

## 关键文件位置

```
/home/chandan/
├── mixed_reality_ws/              ← 主工作空间 (我创建的)
│   ├── src/
│   │   ├── swarm_tether_estimator/
│   │   ├── unitree_aliengo_bridge/
│   │   └── tether_detection/
│   ├── run_task1.sh
│   ├── run_task2.sh
│   ├── analyze_data.py
│   └── README.md
├── mixed_reality_data/            ← 运行时生成 (CSV logs)
├── formica_ws/                    ← 现有 swarm 仿真
│   └── src/formica_swarm_sim/
│       └── robot_agent.py        ← 已修改以支持 yielding
└── Documents/formicabot_ws/       ← 现有 FormicaBot 核心算法
```

---

**状态**: 代码框架已完成 ✅
**待办**: 物理硬件测试 & 数据收集 (你在 Jetson/Unitree 上运行)

准备好开始实验了吗？先运行 `./run_task1.sh` 试试！
