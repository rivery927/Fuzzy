# 无人机仿真平台使用手册

## Alienware 16 Area-51 研究工作站配置文档

**文档版本**: 1.0  
**创建日期**: 2026年1月17日  
**系统用户**: rivery  
**研究方向**: 无人机传感器欺骗/注入实验

---

# 目录

1. [文档概述](#1-文档概述)
2. [硬件配置](#2-硬件配置)
3. [软件架构总览](#3-软件架构总览)
4. [已安装组件详解](#4-已安装组件详解)
5. [系统架构与数据流](#5-系统架构与数据流)
6. [快速启动指南](#6-快速启动指南)
7. [让无人机起飞](#7-让无人机起飞)
8. [添加传感器](#8-添加传感器)
9. [传感器欺骗实验指南](#9-传感器欺骗实验指南)
10. [ROS2 通信详解](#10-ros2-通信详解)
11. [常用命令速查表](#11-常用命令速查表)
12. [故障排除](#12-故障排除)
13. [开发环境建议](#13-开发环境建议)
14. [附录](#14-附录)

---

# 1. 文档概述

## 1.1 本文档的目的

本文档为使用这台 Alienware 16 Area-51 工作站进行无人机仿真研究的所有人员提供完整的技术参考。

## 1.2 研究背景

本工作站配置用于支持**无人机传感器欺骗/注入实验**研究。研究团队由博士后罗绍成(Dr. Shaocheng Luo)指导，主要研究方向包括：

- 向无人机传感器注入伪造数据
- 分析无人机飞控系统对欺骗性传感器数据的响应
- 开发传感器欺骗检测算法
- 评估无人机系统的安全性

---

# 2. 硬件配置

## 2.1 主要硬件规格

| 组件 | 规格 |
|------|------|
| **型号** | Alienware 16 Area-51 (AA16250) |
| **GPU** | NVIDIA GeForce RTX 5080 (16GB GDDR7) |
| **存储** | 2TB NVMe SSD (双系统分区) |

## 2.2 查看 GPU 状态

```bash
nvidia-smi
```

---

# 3. 软件架构总览

## 3.1 各组件版本汇总

| 组件 | 版本 | 安装位置 |
|------|------|----------|
| Ubuntu | 24.04 LTS | / |
| NVIDIA Driver | 580.95.05 | 系统内核模块 |
| CUDA Toolkit | 13.0 | /usr/local/cuda-13.0/ |
| GCC/G++ | 11.5.0 (默认) | /usr/bin/gcc-11 |
| ROS2 | JAZZY | /opt/ros/jazzy/ |
| Isaac Sim | 5.1.0 | ~/isaacsim/ |
| PX4-Autopilot | v1.14.3 | ~/PX4-Autopilot/ |
| Pegasus Simulator | 5.1.0 | ~/PegasusSimulator/ |

## 3.2 软件栈层次结构

```
应用层:     你的研究代码 / QGroundControl
     ↓
通信层:     ROS2 JAZZY (机器人通信中间件)
     ↓
飞控层:     PX4-Autopilot v1.14.3 (无人机飞控软件)
     ↓
仿真层:     Pegasus Simulator (无人机仿真框架)
     ↓
渲染层:     Isaac Sim 5.1.0 (3D 物理仿真器)
     ↓
系统层:     CUDA 13.0 / NVIDIA Driver 580.x / Ubuntu 24.04
```

---

# 4. 已安装组件详解

## 4.1 Ubuntu 24.04 LTS

操作系统，所有软件运行的基础平台。LTS 版本提供 5 年安全更新支持。

## 4.2 NVIDIA 驱动 (580.95.05)

让操作系统能够识别和使用 NVIDIA RTX 5080 显卡的软件。

```bash
# 查看驱动状态
nvidia-smi
```

## 4.3 CUDA Toolkit 13.0

NVIDIA 的并行计算平台，让软件能利用 GPU 进行大规模并行计算。Isaac Sim 用它做物理仿真和渲染加速。

```bash
# 查看 CUDA 版本
nvcc --version
```

## 4.4 GCC 11

C/C++ 编译器。**重要**: Ubuntu 24.04 默认是 GCC 13，但 Isaac Sim 不兼容 GCC 12+，因此我们安装了 GCC 11 并设置为默认。

```bash
# 查看 GCC 版本
gcc --version
```

## 4.5 ROS2 JAZZY

机器人软件开发框架，是各组件之间通信的"总线"。

**核心概念**:
- **节点 (Node)**: 执行特定功能的程序
- **话题 (Topic)**: 节点之间传递消息的"频道"
- **消息 (Message)**: 节点之间传递的数据格式

```bash
# 查看所有话题
ros2 topic list

# 查看特定话题数据
ros2 topic echo /fmu/out/vehicle_status
```

## 4.6 Isaac Sim 5.1.0

NVIDIA 的 3D 机器人仿真器，提供：
- **PhysX 5 物理引擎**: 模拟真实物理规律
- **RTX 光线追踪渲染**: 逼真的视觉效果
- **传感器仿真**: 相机、LiDAR、IMU 等

```bash
# 启动 Isaac Sim
~/isaacsim/isaac-sim.sh

# 使用 Isaac Sim Python
$ISAACSIM_PYTHON your_script.py
```

**注意**: 第一次启动需要 5-15 分钟编译着色器缓存。

## 4.7 PX4-Autopilot v1.14.3

开源无人机飞控软件，负责：
- 读取传感器数据
- 估计飞行状态（EKF2 状态估计器）
- 执行飞行控制算法
- 输出电机控制指令

**SITL 模式**: Software-In-The-Loop，让 PX4 在电脑上运行，接收虚拟传感器数据。

```bash
# 编译并启动 PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl_default none
```

## 4.8 Pegasus Simulator 5.1.0

无人机仿真框架，在 Isaac Sim 和 PX4 之间建立桥梁：
- 在 Isaac Sim 中创建无人机模型
- 模拟传感器数据（IMU、GPS、气压计等）
- 通过 MAVLink 协议与 PX4 通信

---

# 5. 系统架构与数据流

## 5.1 完整数据流

```
Isaac Sim (物理仿真)
    ↓ 物理状态
Pegasus (传感器仿真)
    ↓ HIL_SENSOR / HIL_GPS (MAVLink)
PX4 SITL (飞控)
    ↓ 电机指令 (MAVLink)
Pegasus → Isaac Sim (更新物理状态)

同时:
PX4 SITL → uXRCE-DDS → ROS2 (发布状态话题)
    ↓
你的研究代码 (订阅话题、进行实验)
```

## 5.2 MAVLink 关键消息

| 消息 | 方向 | 内容 |
|------|------|------|
| HIL_SENSOR | Pegasus → PX4 | IMU、气压计、磁力计数据 |
| HIL_GPS | Pegasus → PX4 | GPS 位置和速度 |
| HIL_ACTUATOR_CONTROLS | PX4 → Pegasus | 电机控制指令 |

**HIL_SENSOR 是传感器欺骗的主要攻击目标！**

---

# 6. 快速启动指南

## 6.1 启动完整仿真环境（推荐）

```bash
# 打开终端 (Ctrl + Alt + T)
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py
```

这会自动启动 Isaac Sim + Pegasus + PX4。

## 6.2 验证成功启动

成功后你应该看到：
- Isaac Sim 窗口中有一架四旋翼无人机
- 终端显示 "Received first heartbeat"
- 终端显示 "[commander] Ready for takeoff!"

## 6.3 停止仿真

```bash
# 按 Ctrl + C 停止

# 如果需要强制停止
pkill -9 px4
```

---

# 7. 让无人机起飞

## 7.1 方法一：使用 QGroundControl（图形界面）

### 安装 QGroundControl

```bash
cd ~/Downloads
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```

### 使用步骤

1. 启动仿真环境
2. 运行 `~/Downloads/QGroundControl.AppImage`
3. 等待 QGroundControl 自动连接到 PX4
4. 点击 "Takeoff" 模式
5. 滑动解锁条

## 7.2 方法二：使用 MAVSDK-Python（代码控制）

### 安装

```bash
pip install mavsdk --break-system-packages
```

### 起飞脚本

```python
#!/usr/bin/env python3
import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("等待连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("已连接!")
            break
    
    print("等待 GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("GPS 就绪!")
            break
    
    print("解锁...")
    await drone.action.arm()
    
    print("起飞...")
    await drone.action.takeoff()
    
    await asyncio.sleep(10)
    
    print("降落...")
    await drone.action.land()

asyncio.run(run())
```

---

# 8. 添加传感器

## 8.1 Pegasus 内置传感器

| 传感器 | 更新频率 |
|--------|----------|
| IMU | 250 Hz |
| GPS | 10 Hz |
| Barometer | 50 Hz |
| Magnetometer | 50 Hz |

## 8.2 添加相机

```python
from isaacsim.sensors.camera import Camera

camera = Camera(
    prim_path="/World/quadrotor/front_camera",
    name="front_camera",
    frequency=30,
    resolution=(640, 480),
    translation=(0.1, 0.0, 0.0),
)
camera.initialize()

# 获取图像
rgb_image = camera.get_rgb()
depth_image = camera.get_depth()
```

## 8.3 添加 LiDAR

```python
from isaacsim.sensors.lidar import LiDAR

lidar = LiDAR(
    prim_path="/World/quadrotor/lidar",
    name="lidar",
    frequency=10,
    min_range=0.4,
    max_range=100.0,
    horizontal_fov=360.0,
)
lidar.initialize()

# 获取点云
point_cloud = lidar.get_point_cloud()
```

---

# 9. 传感器欺骗实验指南

## 9.1 什么是传感器欺骗？

向无人机的传感器系统注入虚假数据，使其做出错误的状态估计和飞行决策。

```
正常: 传感器 → 真实数据 → EKF2 → 正确估计 → 正确飞行
欺骗: 传感器 → 攻击者(修改) → EKF2 → 错误估计 → 错误飞行
```

## 9.2 攻击注入点（推荐）

**位置**: `~/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py`

在这里修改 MAVLink 消息，可以精确控制注入的虚假数据。

## 9.3 GPS 欺骗示例

在 `px4_mavlink_backend.py` 中添加：

```python
class PX4MavlinkBackend:
    def __init__(self):
        # 添加欺骗参数
        self.gps_spoofing_enabled = False
        self.gps_offset_lat = 0.0  # 纬度偏移（度）
        self.gps_offset_lon = 0.0  # 经度偏移（度）
    
    def enable_gps_spoofing(self, lat_offset, lon_offset):
        self.gps_spoofing_enabled = True
        self.gps_offset_lat = lat_offset
        self.gps_offset_lon = lon_offset
    
    def send_hil_gps(self, timestamp, lat, lon, alt, ...):
        # 如果启用欺骗，修改数据
        if self.gps_spoofing_enabled:
            lat += self.gps_offset_lat
            lon += self.gps_offset_lon
        
        # 发送消息...
```

## 9.4 IMU 欺骗示例

```python
# 添加 IMU 偏置
self.accel_bias = [0.0, 0.0, 0.0]
self.gyro_bias = [0.0, 0.0, 0.0]

def send_hil_sensor(self, ..., xacc, yacc, zacc, xgyro, ygyro, zgyro, ...):
    if self.imu_spoofing_enabled:
        xacc += self.accel_bias[0]
        yacc += self.accel_bias[1]
        zacc += self.accel_bias[2]
        # ... 类似处理陀螺仪
```

## 9.5 实验流程

1. **正常飞行基线**: 先记录无欺骗时的飞行数据
2. **启用欺骗**: 在特定时间注入虚假数据
3. **记录响应**: 记录真实位置和估计位置
4. **分析结果**: 计算误差、恢复时间等

---

# 10. ROS2 通信详解

## 10.1 PX4 发布的话题

| 话题 | 说明 |
|------|------|
| /fmu/out/vehicle_status | 飞行器状态 |
| /fmu/out/vehicle_local_position | 本地位置 |
| /fmu/out/vehicle_attitude | 姿态 |
| /fmu/out/sensor_combined | 传感器数据 |

## 10.2 PX4 订阅的话题

| 话题 | 说明 |
|------|------|
| /fmu/in/offboard_control_mode | Offboard 模式 |
| /fmu/in/trajectory_setpoint | 轨迹设定点 |
| /fmu/in/vehicle_command | 飞行命令 |

## 10.3 常用命令

```bash
ros2 topic list                              # 列出所有话题
ros2 topic echo /fmu/out/vehicle_status      # 查看数据
ros2 topic hz /fmu/out/vehicle_local_position # 查看频率
```

---

# 11. 常用命令速查表

## 11.1 系统命令

```bash
nvidia-smi          # GPU 状态
htop                # 系统资源
df -h               # 磁盘使用
free -h             # 内存使用
```

## 11.2 仿真启动

```bash
# 完整仿真（推荐）
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py

# 单独启动 Isaac Sim
~/isaacsim/isaac-sim.sh

# 单独启动 PX4
cd ~/PX4-Autopilot
make px4_sitl_default none

# QGroundControl
~/Downloads/QGroundControl.AppImage
```

## 11.3 ROS2 命令

```bash
ros2 topic list
ros2 topic echo /topic_name
ros2 node list
```

## 11.4 PX4 命令 (pxh> 提示符)

```bash
param show EKF2*     # 查看参数
param set NAME VALUE # 设置参数
sensor_status        # 传感器状态
ekf2 status          # EKF2 状态
shutdown             # 退出
```

---

# 12. 故障排除

## 12.1 "Waiting for first heartbeat" 一直显示

```bash
pkill -9 px4
# 重新启动仿真
```

## 12.2 Isaac Sim 启动失败

```bash
nvidia-smi  # 检查驱动
rm -rf ~/.nvidia-omniverse/cache  # 清除缓存
sudo reboot
```

## 12.3 PX4 编译失败

```bash
cd ~/PX4-Autopilot
make clean
pip install kconfiglib future --break-system-packages
make px4_sitl_default none
```

## 12.4 ros2 命令找不到

```bash
source /opt/ros/jazzy/setup.bash
```

---

# 13. 开发环境建议

## 13.1 关于 VS Code

目前没有安装 VS Code。**没有问题**，但建议安装以便更方便地开发。

### 安装 VS Code

```bash
sudo snap install code --classic
```

### 推荐扩展

```bash
code --install-extension ms-python.python
code --install-extension ms-iot.vscode-ros
```

## 13.2 其他推荐工具

```bash
# 多窗口终端
sudo apt install terminator

# 交互式 Python
pip install ipython --break-system-packages
```

---

# 14. 附录

## 14.1 重要文件路径

| 文件/目录 | 路径 |
|-----------|------|
| Isaac Sim | ~/isaacsim/ |
| Isaac Sim Python | ~/isaacsim/python.sh |
| PX4 | ~/PX4-Autopilot/ |
| Pegasus | ~/PegasusSimulator/ |
| Pegasus 配置 | ~/PegasusSimulator/extensions/pegasus.simulator/config/configs.yaml |
| ROS2 | /opt/ros/jazzy/ |
| CUDA | /usr/local/cuda-13.0/ |
| 环境变量 | ~/.bashrc |

## 14.2 ~/.bashrc 中的环境变量

```bash
# CUDA 13.0
export PATH=/usr/local/cuda-13.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH

# ROS2 JAZZY
source /opt/ros/jazzy/setup.bash

# Isaac Sim
export ISAACSIM_PATH="${HOME}/isaacsim"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
alias isaac_run="${ISAACSIM_PATH}/isaac-sim.sh"
```

## 14.3 参考资料

- [Isaac Sim 文档](https://docs.isaacsim.omniverse.nvidia.com/)
- [PX4 文档](https://docs.px4.io/)
- [ROS2 JAZZY 文档](https://docs.ros.org/en/jazzy/)
- [Pegasus Simulator 文档](https://pegasussimulator.github.io/PegasusSimulator/)

---

# 文档更新记录

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| 1.0 | 2026-01-17 | 初始版本 |

---

**文档结束**

*acknowledgment：本文档由 Claude AI 协助生成*
