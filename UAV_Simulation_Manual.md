# 无人机仿真平台使用手册

## Alienware 16 Area-51 研究工作站完整文档

**文档版本**: 2.0  
**创建日期**: 2026年1月17日  
**最后更新**: 2026年1月18日  
**系统用户**: rivery  
**研究方向**: 无人机感知传感器欺骗实验

---

# 目录

1. [文档概述](#1-文档概述)
2. [硬件配置](#2-硬件配置)
3. [软件架构总览](#3-软件架构总览)
4. [已安装组件详解](#4-已安装组件详解)
5. [系统架构与数据流](#5-系统架构与数据流)
6. [快速启动指南](#6-快速启动指南)
7. [让无人机起飞](#7-让无人机起飞)
8. [感知传感器配置](#8-感知传感器配置)
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

本工作站配置用于支持**无人机感知传感器欺骗实验**研究。研究团队由 Dr. Shaocheng Luo 指导。

### 研究重点（根据博士后邮件）

> "We will need to inject spoofed/fake objects/points into these perceptual sensors. That is our research focus."

主要研究方向：
- **感知传感器欺骗**：向相机、LiDAR、毫米波雷达注入虚假物体/点
- 分析无人机感知系统对欺骗数据的响应
- 开发传感器欺骗检测算法
- 评估无人机感知系统的安全性

### 两种传感器欺骗类型对比

| 类型 | 目标传感器 | 攻击效果 | 本研究重点 |
|------|------------|----------|------------|
| **感知传感器欺骗** | 相机、LiDAR、雷达 | 让无人机"看到"虚假物体或"看不到"真实物体 | ✅ **是** |
| 导航传感器欺骗 | GPS、IMU、气压计、磁力计 | 让无人机飞错位置或姿态错误 | 辅助参考 |

---

# 2. 硬件配置

## 2.1 主要硬件规格

| 组件 | 规格 | 说明 |
|------|------|------|
| **型号** | Alienware 16 Area-51 (AA16250) | Dell 高端移动工作站 |
| **处理器** | Intel Core Ultra 9 275HX | 24核心/24线程 |
| **GPU** | NVIDIA GeForce RTX 5080 Laptop | 16GB GDDR7, Blackwell 架构 |
| **内存** | 32GB DDR5 | 足够运行大型仿真 |
| **存储** | 2TB NVMe SSD | 双系统分区 |
| **显示** | 集成 Intel ARL GPU + 独立 RTX 5080 | 仿真使用独立 GPU |

## 2.2 查看硬件状态

```bash
# GPU 状态
nvidia-smi

# CPU 信息
lscpu | head -20

# 内存使用
free -h

# 磁盘使用
df -h
```

## 2.3 GPU 状态示例输出

```
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 580.95.05              Driver Version: 580.95.05      CUDA Version: 13.0     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 5080 ...    Off | 00000000:02:00.0  On |                  N/A |
| N/A   45C    P8              15W / 175W |     502MiB / 16303MiB |      2%      Default |
+-----------------------------------------+------------------------+----------------------+
```

---

# 3. 软件架构总览

## 3.1 各组件版本汇总

| 组件 | 版本 | 安装位置 | 用途 |
|------|------|----------|------|
| Ubuntu | 24.04 LTS | / | 操作系统基础 |
| NVIDIA Driver | 580.95.05 | 系统内核模块 | GPU 驱动 |
| CUDA Toolkit | 13.0 | /usr/local/cuda-13.0/ | GPU 计算库 |
| GCC/G++ | 11.5.0 (默认) | /usr/bin/gcc-11 | 编译器 |
| ROS2 | JAZZY | /opt/ros/jazzy/ | 机器人通信框架 |
| Isaac Sim | 5.1.0 | ~/isaacsim/ | 3D 物理仿真 |
| PX4-Autopilot | **v1.15.4** | ~/PX4-Autopilot/ | 无人机飞控 |
| Pegasus Simulator | 5.1.0 | ~/PegasusSimulator/ | 无人机仿真框架 |

## 3.2 软件栈层次结构

```
┌─────────────────────────────────────────────────────────────────────┐
│                         应用层                                       │
│         你的研究代码 / QGroundControl / MAVSDK                      │
├─────────────────────────────────────────────────────────────────────┤
│                         通信层                                       │
│              ROS2 JAZZY (机器人通信中间件)                           │
├─────────────────────────────────────────────────────────────────────┤
│                         飞控层                                       │
│           PX4-Autopilot v1.15.4 (无人机飞控软件)                    │
│                 ├── EKF2 状态估计器                                  │
│                 ├── 位置/姿态控制器                                  │
│                 └── 混控器                                           │
├─────────────────────────────────────────────────────────────────────┤
│                         仿真层                                       │
│         Pegasus Simulator 5.1.0 (无人机仿真框架)                    │
│                 ├── 传感器模型 (IMU/GPS/气压计)                      │
│                 ├── MAVLink 通信                                     │
│                 └── 感知传感器 (相机/LiDAR)                          │
├─────────────────────────────────────────────────────────────────────┤
│                         渲染层                                       │
│            Isaac Sim 5.1.0 (3D 物理仿真器)                          │
│                 ├── PhysX 5 物理引擎                                 │
│                 ├── RTX 光线追踪渲染                                 │
│                 └── USD 场景管理                                     │
├─────────────────────────────────────────────────────────────────────┤
│                         系统层                                       │
│      CUDA 13.0 / NVIDIA Driver 580.x / Ubuntu 24.04 LTS            │
└─────────────────────────────────────────────────────────────────────┘
```

## 3.3 组件间数据流频率

| 数据流 | 频率 | 协议/接口 |
|--------|------|-----------|
| 物理仿真步进 | 250 Hz | PhysX |
| IMU 数据 | 250 Hz | MAVLink HIL_SENSOR |
| GPS 数据 | 10 Hz | MAVLink HIL_GPS |
| 气压计 | 50 Hz | MAVLink HIL_SENSOR |
| 相机图像 | 30 Hz (可配置) | Isaac Sim API |
| LiDAR 点云 | 10 Hz (可配置) | Isaac Sim API |
| EKF2 状态输出 | 250 Hz | uORB/ROS2 |
| 电机控制指令 | 250 Hz | MAVLink HIL_ACTUATOR |

---

# 4. 已安装组件详解

## 4.1 Ubuntu 24.04 LTS

操作系统基础，所有软件运行的平台。

**为什么选择 Ubuntu 而不是 Windows？**
- ROS2 JAZZY 原生支持
- Linux 可配置实时内核，机器人控制需要毫秒级响应
- 大多数机器人软件都是为 Linux 开发的
- 与真实无人机的飞控电脑环境一致

```bash
# 查看系统版本
lsb_release -a
```

## 4.2 NVIDIA 驱动 (580.95.05)

显卡驱动是操作系统与 GPU 之间的"翻译官"。

**为什么是 580.x 版本？**
- RTX 5080 是 Blackwell 架构（NVIDIA 最新一代）
- 需要 570+ 版本驱动才能支持

```bash
# 查看驱动状态
nvidia-smi

# 查看详细驱动信息
nvidia-smi -q | head -50
```

## 4.3 CUDA Toolkit 13.0

NVIDIA 的并行计算平台，让程序能利用 GPU 的数千个计算核心。

**在仿真中的作用**：
- PhysX 物理仿真加速
- RTX 光线追踪渲染
- LiDAR 点云生成
- 相机图像渲染

```bash
# 查看 CUDA 版本
nvcc --version

# 查看 CUDA 安装路径
ls /usr/local/cuda-13.0/
```

## 4.4 GCC 11

C/C++ 编译器，将源代码转换成可执行程序。

**重要**: Ubuntu 24.04 默认是 GCC 13，但 Isaac Sim 5.1.0 使用 GCC 11 编译，与 GCC 12+ 不兼容，因此必须降级。

```bash
# 查看当前 GCC 版本
gcc --version

# 如需切换版本
sudo update-alternatives --config gcc
```

## 4.5 ROS2 JAZZY

机器人操作系统 2 是机器人软件之间的"微信群"——不同程序通过 ROS2 互相发送消息。

### 核心概念

| 概念 | 说明 | 类比 |
|------|------|------|
| **节点 (Node)** | 独立运行的程序 | 群成员 |
| **话题 (Topic)** | 消息传递的频道 | 群聊 |
| **消息 (Message)** | 节点间传递的数据 | 聊天内容 |
| **发布者 (Publisher)** | 发送消息的节点 | 发消息的人 |
| **订阅者 (Subscriber)** | 接收消息的节点 | 看消息的人 |

### 发布-订阅模式

```
                    话题: /drone/position
                          │
    ┌─────────────────────┼─────────────────────┐
    │                     │                     │
    ▼                     │                     ▼
┌──────┐            ┌────┴────┐            ┌──────┐
│订阅者│            │ 发布者  │            │订阅者│
│ GUI  │            │  PX4    │            │你的代码│
└──────┘            └─────────┘            └──────┘
```

```bash
# 查看所有话题
ros2 topic list

# 查看特定话题数据
ros2 topic echo /fmu/out/vehicle_status

# 查看话题更新频率
ros2 topic hz /fmu/out/vehicle_local_position
```

## 4.6 Isaac Sim 5.1.0

NVIDIA 的专业 3D 机器人仿真器，创建逼真的虚拟世界。

### 核心组件

| 组件 | 功能 |
|------|------|
| **PhysX 5** | 物理引擎，模拟真实物理规律（碰撞、重力、动力学） |
| **RTX 渲染器** | 实时光线追踪，生成逼真图像 |
| **USD** | 通用场景描述格式，管理 3D 场景 |
| **Omniverse Kit** | 应用框架和扩展系统 |

### 传感器仿真能力

| 传感器类型 | 支持情况 | 配置位置 |
|------------|----------|----------|
| RGB 相机 | ✅ | Isaac Sim API |
| 深度相机 | ✅ | Isaac Sim API |
| LiDAR | ✅ | ~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/ |
| IMU | ✅ | Pegasus 内置 |
| GPS | ✅ | Pegasus 内置 |
| 毫米波雷达 | ✅ | Isaac Sim API |

```bash
# 启动 Isaac Sim GUI
~/isaacsim/isaac-sim.sh

# 使用 Isaac Sim Python 运行脚本
$ISAACSIM_PYTHON your_script.py
```

**注意**: 第一次启动需要 5-15 分钟编译着色器缓存（shader warmup）。

## 4.7 PX4-Autopilot v1.15.4

世界上最流行的开源无人机飞控软件，是无人机的"大脑"。

### 核心模块

```
┌─────────────────────────────────────────────────────────────────────┐
│                      PX4 内部架构                                    │
├─────────────────────────────────────────────────────────────────────┤
│  传感器驱动层                                                        │
│  ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐                           │
│  │ IMU │ │ GPS │ │气压计│ │磁力计│ │光流 │                           │
│  └──┬──┘ └──┬──┘ └──┬──┘ └──┬──┘ └──┬──┘                           │
│     └───────┴───────┴───────┴───────┘                               │
│                      │                                               │
│                      ▼                                               │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    EKF2 状态估计器                           │   │
│  │                                                             │   │
│  │  ★ 融合多个传感器数据，估计无人机完整状态 ★                 │   │
│  │  输出：位置、速度、姿态、角速度、偏置估计                    │   │
│  │                                                             │   │
│  └────────────────────────┬────────────────────────────────────┘   │
│                           │                                         │
│                           ▼                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │ 位置控制器   │→│ 姿态控制器   │→│ 角速度控制器 │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
│                                              │                      │
│                                              ▼                      │
│                                      ┌──────────────┐              │
│                                      │   混控器     │              │
│                                      │ 输出电机PWM  │              │
│                                      └──────────────┘              │
└─────────────────────────────────────────────────────────────────────┘
```

### SITL 模式

**SITL (Software-In-The-Loop)**: 让 PX4 在电脑上运行，接收虚拟传感器数据。

```
真实飞行:  真实传感器 → 飞控板(Pixhawk) → 真实电机
SITL仿真:  Pegasus仿真 → 你的电脑(PX4 SITL) → Pegasus仿真
```

```bash
# 编译并启动 PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl_default none

# 在 pxh> 提示符下
commander status     # 查看状态
ekf2 status         # EKF2 状态
param show EKF2*    # 查看 EKF2 参数
shutdown            # 退出
```

## 4.8 Pegasus Simulator 5.1.0

专门为无人机仿真设计的框架，是 **Isaac Sim 和 PX4 之间的桥梁**。

### 核心功能

| 功能 | 说明 |
|------|------|
| 无人机模型管理 | 在 Isaac Sim 中创建和配置无人机 |
| 传感器仿真 | 模拟 IMU、GPS、气压计、磁力计数据 |
| MAVLink 通信 | 与 PX4 SITL 通信 |
| 图形传感器 | 相机、LiDAR 配置 |
| ROS2 集成 | 发布传感器数据到 ROS2 话题 |

### 传感器仿真流程

```
Isaac Sim 物理状态 (真实值)
        │
        ▼
┌─────────────────────────────────────┐
│     Pegasus 传感器模型              │
│                                     │
│  ┌─────────────────────────────┐   │
│  │ IMU 模型                     │   │
│  │ 真实值 + 偏置 + 噪声 → 输出 │   │
│  └─────────────────────────────┘   │
│                                     │
│  ┌─────────────────────────────┐   │
│  │ GPS 模型                     │   │
│  │ 真实值 + 噪声 → 输出        │   │
│  └─────────────────────────────┘   │
│                                     │
└─────────────────────────────────────┘
        │
        ▼ MAVLink HIL_SENSOR / HIL_GPS
        │
    PX4 SITL
```

---

# 5. 系统架构与数据流

## 5.1 完整系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           完整系统架构                                        │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      Isaac Sim 5.1.0                                 │   │
│  │                                                                     │   │
│  │   ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐    │   │
│  │   │  PhysX 5    │  │ RTX Renderer│  │   传感器仿真             │    │   │
│  │   │  物理引擎   │  │  光线追踪   │  │  相机/LiDAR/雷达        │    │   │
│  │   └──────┬──────┘  └──────┬──────┘  └───────────┬─────────────┘    │   │
│  │          │                │                     │                   │   │
│  └──────────┼────────────────┼─────────────────────┼───────────────────┘   │
│             │                │                     │                       │
│             ▼                ▼                     ▼                       │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Pegasus Simulator 5.1.0                           │   │
│  │                                                                     │   │
│  │   ┌─────────────────┐      ┌─────────────────────────────────┐     │   │
│  │   │  无人机模型     │      │  px4_mavlink_backend.py         │     │   │
│  │   │  (Iris 四旋翼)  │      │  ★ 传感器数据通道 ★             │     │   │
│  │   └────────┬────────┘      │  ★ 攻击注入点 ★                 │     │   │
│  │            │               └──────────────┬──────────────────┘     │   │
│  │            │                              │                         │   │
│  └────────────┼──────────────────────────────┼─────────────────────────┘   │
│               │                              │ MAVLink                     │
│               │                              │ (TCP 4560)                  │
│               ▼                              ▼                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      PX4 SITL v1.15.4                                │   │
│  │                                                                     │   │
│  │   ┌─────────────────┐      ┌─────────────────────────────────┐     │   │
│  │   │     EKF2        │      │      控制器级联                  │     │   │
│  │   │  状态估计器     │─────▶│  位置→姿态→角速度→电机          │     │   │
│  │   │                 │      └──────────────────────────────────┘     │   │
│  │   └─────────────────┘                                               │   │
│  │                                                                     │   │
│  │   uXRCE-DDS Agent (UDP 8888)                                        │   │
│  │            │                                                        │   │
│  └────────────┼────────────────────────────────────────────────────────┘   │
│               │                                                            │
│               ▼                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      ROS2 JAZZY                                      │   │
│  │                                                                     │   │
│  │   发布话题:                                                          │   │
│  │   /fmu/out/vehicle_local_position   (位置估计)                       │   │
│  │   /fmu/out/vehicle_attitude         (姿态估计)                       │   │
│  │   /fmu/out/sensor_combined          (传感器数据)                     │   │
│  │   /fmu/out/estimator_status         (EKF2 状态)                      │   │
│  │                                                                     │   │
│  │   订阅话题:                                                          │   │
│  │   /fmu/in/trajectory_setpoint       (轨迹设定点)                     │   │
│  │   /fmu/in/vehicle_command           (飞行命令)                       │   │
│  │                                                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│               │                                                            │
│               ▼                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                       你的研究代码                                   │   │
│  │                                                                     │   │
│  │   - 订阅传感器数据和状态估计                                         │   │
│  │   - 发送控制指令                                                     │   │
│  │   - 实施欺骗攻击                                                     │   │
│  │   - 分析攻击效果                                                     │   │
│  │                                                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 5.2 一个仿真周期的数据流

```
时间戳 T 的完整数据流:

1. 物理仿真 (Isaac Sim)
   └─▶ 计算螺旋桨推力/扭矩 → 牛顿-欧拉方程 → 更新位置/速度/姿态
       输出: 真实状态 (Ground Truth)

2. 传感器仿真 (Pegasus)
   └─▶ 真实状态 + 传感器模型(偏置/噪声) → 虚拟传感器读数
       输出: HIL_SENSOR (IMU), HIL_GPS (GPS)

3. 状态估计 (PX4 EKF2)
   └─▶ 预测(IMU积分) + 更新(GPS/气压计修正) → 状态估计
       输出: 估计位置/速度/姿态

4. 控制器 (PX4)
   └─▶ 目标位置 - 估计位置 = 误差 → PID控制 → 电机指令
       输出: HIL_ACTUATOR_CONTROLS

5. 执行器仿真 (Pegasus)
   └─▶ PWM → 电机转速 → 螺旋桨推力
       输出: 推力和扭矩 → 返回步骤1

整个循环: 250 Hz (每 4 毫秒一次)
```

## 5.3 MAVLink 关键消息

| 消息 | 方向 | 频率 | 内容 |
|------|------|------|------|
| HIL_SENSOR | Pegasus → PX4 | 250 Hz | 加速度、角速度、磁场、气压 |
| HIL_GPS | Pegasus → PX4 | 10 Hz | 经纬度、高度、速度、精度 |
| HIL_ACTUATOR_CONTROLS | PX4 → Pegasus | 250 Hz | 4个电机的 PWM 值 |

---

# 6. 快速启动指南

## 6.1 启动完整仿真环境（推荐方式）

```bash
# 打开终端 (Ctrl + Alt + T)
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py
```

这会自动启动：
- Isaac Sim 渲染窗口
- Pegasus 无人机模型
- PX4 SITL 飞控

## 6.2 启动带传感器的仿真

```bash
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/sensor_test.py
```

包含：
- 前置 RGB+深度相机
- LiDAR
- 3个彩色障碍物（用于感知测试）

## 6.3 验证成功启动

成功后你应该看到：

**Isaac Sim 窗口**：
- 3D 场景中有一架四旋翼无人机
- 蓝色网格地面

**终端输出**：
```
INFO  [simulator_mavlink] Waiting for first heartbeat
INFO  [simulator_mavlink] Received first heartbeat     ← 关键！
INFO  [commander] Ready for takeoff!                   ← 关键！
```

## 6.4 停止仿真

```bash
# 方法1：按 Ctrl + C

# 方法2：如果卡住，强制停止
pkill -9 px4
pkill -9 isaac
```

## 6.5 首次启动注意事项

| 情况 | 说明 | 解决方案 |
|------|------|----------|
| 启动很慢 (5-15分钟) | 首次编译着色器缓存 | 正常，耐心等待 |
| 黑屏或崩溃 | 着色器缓存损坏 | 清除缓存后重试 |
| "Waiting for heartbeat" 卡住 | 上次 PX4 进程未退出 | `pkill -9 px4` |

---

# 7. 让无人机起飞

## 7.1 方法一：使用 QGroundControl（图形界面，推荐新手）

### 安装 QGroundControl

```bash
cd ~/Downloads
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```

### 使用步骤

1. 先启动仿真环境
2. 运行 QGroundControl：
   ```bash
   ~/Downloads/QGroundControl.AppImage
   ```
3. 等待 QGroundControl 自动连接到 PX4（左上角显示 "Connected"）
4. 点击左侧飞行模式按钮，选择 "Takeoff"
5. 滑动底部解锁条确认起飞

## 7.2 方法二：使用 MAVSDK-Python（代码控制，推荐研究使用）

### 安装

```bash
pip install mavsdk --break-system-packages
```

### 完整起飞脚本

```python
#!/usr/bin/env python3
"""
simple_takeoff.py - 简单的无人机起飞测试
"""
import asyncio
from mavsdk import System

async def run():
    # 创建无人机对象
    drone = System()
    
    # 连接到 PX4 (默认端口 14540)
    await drone.connect(system_address="udp://:14540")
    
    # 等待连接
    print("等待连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"✓ 已连接到无人机!")
            break
    
    # 等待 GPS 定位
    print("等待 GPS 定位...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✓ GPS 定位完成!")
            break
    
    # 解锁电机
    print("解锁中...")
    await drone.action.arm()
    print("✓ 已解锁!")
    
    # 起飞到 5 米高度
    print("起飞...")
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()
    
    # 悬停 10 秒
    print("悬停 10 秒...")
    await asyncio.sleep(10)
    
    # 降落
    print("降落...")
    await drone.action.land()
    
    # 等待降落完成
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("✓ 已降落!")
            break

if __name__ == "__main__":
    asyncio.run(run())
```

### 运行

```bash
# 先启动仿真
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py &

# 等待 "Ready for takeoff" 后，运行起飞脚本
python3 simple_takeoff.py
```

## 7.3 方法三：使用 PX4 命令行

在 PX4 的 `pxh>` 提示符下：

```bash
# 解锁
commander arm

# 起飞（需要先设置模式）
commander takeoff

# 降落
commander land

# 紧急停止
commander disarm -f
```

## 7.4 方法四：使用 ROS2 话题

```bash
# 发送起飞命令
ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{command: 22, param1: 0.0, param2: 0.0, target_system: 1, target_component: 1}"
```

---

# 8. 感知传感器配置

## 8.1 博士后研究要求

根据 Dr. Shaocheng Luo 的邮件：

> "For the simulated drone, we need to have a camera, lidar, and possibly mmwave radar perception, all first-person view, from the simulator."

### 需要配置的传感器

| 传感器 | 用途 | 攻击方式 |
|--------|------|----------|
| **相机** | 视觉感知、目标检测 | 注入虚假物体到图像 |
| **LiDAR** | 3D 环境感知、避障 | 注入虚假点到点云 |
| **毫米波雷达** | 测距、测速 | 注入虚假目标回波 |

## 8.2 已配置的传感器测试脚本

位置：`~/PegasusSimulator/examples/sensor_test.py`

### 前置相机配置

```python
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

front_camera = MonocularCamera(
    "front_camera",
    config={
        "frequency": 30.0,              # 30 Hz
        "resolution": (640, 480),       # VGA 分辨率
        "position": np.array([0.15, 0.0, 0.0]),  # 无人机前方 15cm
        "orientation": np.array([0.0, 0.0, 0.0]), # 朝前
        "depth": True,                  # 启用深度图
    }
)
```

### LiDAR 配置

```python
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar

lidar_sensor = Lidar(
    "lidar",
    config={
        "frequency": 10.0,              # 10 Hz
        "position": np.array([0.0, 0.0, 0.05]),  # 无人机顶部 5cm
        "orientation": np.array([0.0, 0.0, 0.0]),
        "sensor_configuration": {"sensor_configuration": "Example_Rotary"},
        "show_render": True,            # 显示点云
    }
)
```

## 8.3 可用的 LiDAR 配置

Isaac Sim 支持多种真实 LiDAR 型号：

| 品牌 | 型号 | 配置文件位置 |
|------|------|--------------|
| **Velodyne** | VLS128 | ~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/Velodyne/ |
| **Ouster** | OS0, OS1, OS2 | ~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/Ouster/ |
| **HESAI** | 多种型号 | ~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/HESAI/ |
| **SICK** | 多种型号 | ~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/SICK/ |

### Ouster OS1 配置示例

```python
lidar_sensor = Lidar(
    "lidar",
    config={
        "frequency": 10.0,
        "position": np.array([0.0, 0.0, 0.05]),
        "sensor_configuration": {
            "sensor_configuration": "OS1_REV6_128ch10hz1024res"  # 128线 Ouster
        },
        "show_render": True,
    }
)
```

## 8.4 运行传感器测试

```bash
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/sensor_test.py
```

**预期结果**：
- 场景中有红、蓝、绿三个方块（障碍物）
- 无人机配备相机和 LiDAR
- 终端显示 "Ready for takeoff"

## 8.5 添加相机到无人机配置

完整示例：

```python
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar

# 创建配置
config = MultirotorConfig()

# 添加相机
front_camera = MonocularCamera("front_camera", config={
    "frequency": 30.0,
    "resolution": (640, 480),
    "position": np.array([0.15, 0.0, 0.0]),
    "depth": True,
})

# 添加 LiDAR
lidar = Lidar("lidar", config={
    "frequency": 10.0,
    "position": np.array([0.0, 0.0, 0.05]),
    "sensor_configuration": {"sensor_configuration": "Example_Rotary"},
})

# 添加到配置
config.graphical_sensors = [front_camera, lidar]

# 创建无人机
Multirotor("/World/quadrotor", ROBOTS['Iris'], 0, [0,0,0.07], ..., config=config)
```

---

# 9. 传感器欺骗实验指南

## 9.1 两类传感器欺骗

### 感知传感器欺骗（研究重点）

| 传感器 | 攻击方式 | 效果 |
|--------|----------|------|
| **相机** | 向图像注入虚假物体 | 无人机"看到"不存在的障碍物 |
| **LiDAR** | 向点云注入虚假点 | 无人机"探测到"虚假障碍物 |
| **雷达** | 注入虚假回波 | 无人机测距/测速错误 |

### 导航传感器欺骗（辅助参考）

| 传感器 | 攻击方式 | 效果 |
|--------|----------|------|
| GPS | 修改位置数据 | 无人机飞错位置 |
| IMU | 添加加速度/角速度偏置 | 无人机姿态错误 |
| 气压计 | 修改气压值 | 高度估计错误 |

## 9.2 感知传感器欺骗实现

### 相机欺骗

**方式1：图像后处理**

```python
import cv2
import numpy as np

def inject_fake_obstacle(image, position, size, color):
    """
    向相机图像注入虚假障碍物
    
    Args:
        image: 原始 RGB 图像
        position: (x, y) 像素坐标
        size: (width, height) 尺寸
        color: (B, G, R) 颜色
    
    Returns:
        修改后的图像
    """
    spoofed_image = image.copy()
    x, y = position
    w, h = size
    
    # 绘制矩形障碍物
    cv2.rectangle(spoofed_image, (x, y), (x+w, y+h), color, -1)
    
    return spoofed_image

# 使用示例
original_image = camera.get_rgb()
spoofed_image = inject_fake_obstacle(
    original_image, 
    position=(320, 240),  # 图像中心
    size=(100, 150),      # 100x150 像素
    color=(0, 0, 255)     # 红色
)
```

**方式2：场景注入（更真实）**

```python
from omni.isaac.core.objects import DynamicCuboid

# 在 Isaac Sim 场景中动态添加虚假物体
fake_obstacle = DynamicCuboid(
    prim_path="/World/fake_obstacle",
    name="fake_obstacle",
    position=np.array([3.0, 0.0, 1.0]),  # 无人机前方 3 米
    scale=np.array([1.0, 1.0, 2.0]),
    color=np.array([255, 0, 0]),  # 红色
)

# 相机会"真实地"看到这个物体
# 这模拟了光学欺骗（如投影攻击）
```

### LiDAR 欺骗

```python
import numpy as np

def inject_fake_points(point_cloud, fake_points):
    """
    向 LiDAR 点云注入虚假点
    
    Args:
        point_cloud: 原始点云 (N, 3)
        fake_points: 要注入的虚假点 (M, 3)
    
    Returns:
        欺骗后的点云 (N+M, 3)
    """
    return np.vstack([point_cloud, fake_points])

def generate_fake_wall(x, y_range, z_range, density=100):
    """
    生成虚假墙壁点云
    
    Args:
        x: 墙壁的 X 坐标
        y_range: (y_min, y_max)
        z_range: (z_min, z_max)
        density: 点密度
    
    Returns:
        虚假点云 (density, 3)
    """
    y = np.random.uniform(y_range[0], y_range[1], density)
    z = np.random.uniform(z_range[0], z_range[1], density)
    x = np.full(density, x)
    
    return np.column_stack([x, y, z])

def remove_real_obstacle(point_cloud, region):
    """
    从点云中移除真实障碍物的点（隐藏攻击）
    
    Args:
        point_cloud: 原始点云
        region: (x_min, x_max, y_min, y_max, z_min, z_max)
    
    Returns:
        移除后的点云
    """
    x_min, x_max, y_min, y_max, z_min, z_max = region
    
    mask = ~(
        (point_cloud[:, 0] >= x_min) & (point_cloud[:, 0] <= x_max) &
        (point_cloud[:, 1] >= y_min) & (point_cloud[:, 1] <= y_max) &
        (point_cloud[:, 2] >= z_min) & (point_cloud[:, 2] <= z_max)
    )
    
    return point_cloud[mask]

# 使用示例
original_cloud = lidar.get_point_cloud()

# 注入虚假墙壁
fake_wall = generate_fake_wall(x=5.0, y_range=(-2, 2), z_range=(0, 2))
spoofed_cloud = inject_fake_points(original_cloud, fake_wall)

# 或者隐藏真实障碍物
spoofed_cloud = remove_real_obstacle(original_cloud, region=(4, 6, -1, 1, 0, 3))
```

## 9.3 导航传感器欺骗实现（参考）

### 攻击注入点

**文件位置**: `~/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py`

### GPS 欺骗框架

```python
class GPSSpoofingModule:
    """GPS 欺骗模块"""
    
    def __init__(self):
        self.enabled = False
        self.lat_offset = 0.0  # 纬度偏移（度）
        self.lon_offset = 0.0  # 经度偏移（度）
        self.alt_offset = 0.0  # 高度偏移（米）
        self.drift_rate = 0.0  # 漂移率
        self.attack_start_time = None
    
    def enable(self, lat_offset=0.0, lon_offset=0.0, alt_offset=0.0, 
               drift_rate=0.0, mode='instant'):
        """
        启用 GPS 欺骗
        
        Args:
            lat_offset: 纬度偏移（度）, 0.0001° ≈ 11米
            lon_offset: 经度偏移（度）
            alt_offset: 高度偏移（米）
            drift_rate: 漂移率（度/秒）
            mode: 'instant' 或 'gradual'
        """
        self.enabled = True
        self.lat_offset = lat_offset
        self.lon_offset = lon_offset
        self.alt_offset = alt_offset
        self.drift_rate = drift_rate
        self.mode = mode
        self.attack_start_time = time.time()
    
    def spoof(self, lat, lon, alt):
        """应用 GPS 欺骗"""
        if not self.enabled:
            return lat, lon, alt
        
        elapsed = time.time() - self.attack_start_time
        
        if self.mode == 'gradual':
            # 渐进式（更隐蔽）
            factor = 1 - np.exp(-elapsed / 5.0)  # 5秒时间常数
            lat_off = self.lat_offset * factor
            lon_off = self.lon_offset * factor
            alt_off = self.alt_offset * factor
        else:
            lat_off = self.lat_offset
            lon_off = self.lon_offset
            alt_off = self.alt_offset
        
        # 添加漂移
        lat_off += self.drift_rate * elapsed
        lon_off += self.drift_rate * elapsed
        
        return lat + lat_off, lon + lon_off, alt + alt_off
```

### IMU 欺骗框架

```python
class IMUSpoofingModule:
    """IMU 欺骗模块"""
    
    def __init__(self):
        self.enabled = False
        self.accel_bias = [0.0, 0.0, 0.0]  # m/s²
        self.gyro_bias = [0.0, 0.0, 0.0]   # rad/s
        self.accel_scale = [1.0, 1.0, 1.0]
        self.gyro_scale = [1.0, 1.0, 1.0]
    
    def enable(self, accel_bias=None, gyro_bias=None):
        """
        启用 IMU 欺骗
        
        典型攻击值：
        - 小偏置: accel_bias=[0.1, 0.0, 0.0] 使无人机缓慢漂移
        - 大偏置: accel_bias=[0.5, 0.0, 0.0] 使无人机快速偏离
        """
        self.enabled = True
        if accel_bias:
            self.accel_bias = accel_bias
        if gyro_bias:
            self.gyro_bias = gyro_bias
    
    def spoof(self, xacc, yacc, zacc, xgyro, ygyro, zgyro):
        """应用 IMU 欺骗"""
        if not self.enabled:
            return xacc, yacc, zacc, xgyro, ygyro, zgyro
        
        # 应用缩放
        xacc *= self.accel_scale[0]
        yacc *= self.accel_scale[1]
        zacc *= self.accel_scale[2]
        
        # 应用偏置
        xacc += self.accel_bias[0]
        yacc += self.accel_bias[1]
        zacc += self.accel_bias[2]
        
        xgyro += self.gyro_bias[0]
        ygyro += self.gyro_bias[1]
        zgyro += self.gyro_bias[2]
        
        return xacc, yacc, zacc, xgyro, ygyro, zgyro
```

## 9.4 实验流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                    传感器欺骗实验流程                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  阶段1: 基线测试（无攻击）                                           │
│  ────────────────────────                                           │
│  1. 启动仿真                                                        │
│  2. 让无人机执行预定义任务（起飞、悬停、航点飞行）                   │
│  3. 记录：                                                          │
│     - 真实状态（从 Isaac Sim）                                      │
│     - 估计状态（从 PX4 EKF2）                                       │
│     - 原始传感器数据                                                │
│     - 感知系统输出                                                  │
│  4. 这是对比的基准                                                  │
│                                                                     │
│                           ↓                                         │
│                                                                     │
│  阶段2: 攻击测试                                                     │
│  ──────────────                                                     │
│  1. 启动仿真                                                        │
│  2. 让无人机开始执行相同任务                                        │
│  3. 在特定时刻启用攻击：                                            │
│     - 相机：注入虚假障碍物                                          │
│     - LiDAR：注入虚假点云                                           │
│  4. 记录相同的数据                                                  │
│  5. 观察：                                                          │
│     - 感知系统是否被欺骗                                            │
│     - 无人机行为如何改变                                            │
│     - 是否触发避障或其他响应                                        │
│                                                                     │
│                           ↓                                         │
│                                                                     │
│  阶段3: 数据分析                                                     │
│  ──────────────                                                     │
│  计算指标：                                                          │
│  - 检测成功率：感知系统检测到虚假物体的次数 / 总注入次数            │
│  - 误报率：将虚假物体识别为真实物体的次数                           │
│  - 响应时间：从注入到无人机响应的时间                               │
│  - 行为偏差：无人机实际路径 vs 预期路径                             │
│                                                                     │
│  可视化：                                                            │
│  - 原始图像 vs 欺骗图像                                             │
│  - 原始点云 vs 欺骗点云                                             │
│  - 无人机轨迹对比                                                   │
│                                                                     │
│                           ↓                                         │
│                                                                     │
│  阶段4: 参数扫描                                                     │
│  ──────────────                                                     │
│  变化攻击参数，研究：                                                │
│  - 虚假物体大小的影响                                               │
│  - 虚假物体位置的影响                                               │
│  - 注入点数量的影响                                                 │
│  - 攻击时机的影响                                                   │
│                                                                     │
│                           ↓                                         │
│                                                                     │
│  阶段5: 防御研究                                                     │
│  ──────────────                                                     │
│  研究如何检测和防御这些攻击：                                        │
│  - 多传感器一致性检测                                               │
│  - 异常检测算法                                                     │
│  - 机器学习检测方法                                                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## 9.5 数据记录框架

```python
import csv
import time
import numpy as np

class ExperimentLogger:
    """实验数据记录器"""
    
    def __init__(self, filename):
        self.filename = filename
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        # 写入表头
        self.writer.writerow([
            'timestamp',
            # 真实状态
            'real_x', 'real_y', 'real_z',
            'real_vx', 'real_vy', 'real_vz',
            # 估计状态
            'est_x', 'est_y', 'est_z',
            'est_vx', 'est_vy', 'est_vz',
            # 感知数据
            'detected_obstacles',
            'lidar_points_count',
            # 攻击状态
            'attack_enabled',
            'attack_type',
            'injected_points_count',
        ])
    
    def log(self, real_state, est_state, perception, attack_info):
        """记录一帧数据"""
        row = [
            time.time(),
            # 真实状态
            real_state['x'], real_state['y'], real_state['z'],
            real_state['vx'], real_state['vy'], real_state['vz'],
            # 估计状态
            est_state['x'], est_state['y'], est_state['z'],
            est_state['vx'], est_state['vy'], est_state['vz'],
            # 感知数据
            perception['detected_obstacles'],
            perception['lidar_points_count'],
            # 攻击状态
            attack_info['enabled'],
            attack_info['type'],
            attack_info['injected_count'],
        ]
        self.writer.writerow(row)
    
    def close(self):
        self.file.close()
```

---

# 10. ROS2 通信详解

## 10.1 PX4 发布的话题（可订阅）

| 话题 | 消息类型 | 频率 | 说明 |
|------|----------|------|------|
| /fmu/out/vehicle_status | VehicleStatus | 1 Hz | 飞行器状态 |
| /fmu/out/vehicle_local_position | VehicleLocalPosition | 50 Hz | 本地位置估计 |
| /fmu/out/vehicle_global_position | VehicleGlobalPosition | 50 Hz | 全局位置估计 |
| /fmu/out/vehicle_attitude | VehicleAttitude | 100 Hz | 姿态估计 |
| /fmu/out/sensor_combined | SensorCombined | 100 Hz | 融合传感器数据 |
| /fmu/out/estimator_status | EstimatorStatus | 1 Hz | EKF2 状态 |

## 10.2 PX4 订阅的话题（可发布）

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| /fmu/in/offboard_control_mode | OffboardControlMode | Offboard 模式设置 |
| /fmu/in/trajectory_setpoint | TrajectorySetpoint | 轨迹设定点 |
| /fmu/in/vehicle_command | VehicleCommand | 飞行命令 |
| /fmu/in/vehicle_rates_setpoint | VehicleRatesSetpoint | 角速度设定点 |

## 10.3 常用 ROS2 命令

```bash
# 列出所有话题
ros2 topic list

# 查看话题数据
ros2 topic echo /fmu/out/vehicle_local_position

# 查看话题频率
ros2 topic hz /fmu/out/vehicle_local_position

# 查看话题类型
ros2 topic type /fmu/out/vehicle_local_position

# 查看消息定义
ros2 interface show px4_msgs/msg/VehicleLocalPosition

# 列出所有节点
ros2 node list

# 查看节点信息
ros2 node info /fmu
```

## 10.4 Python ROS2 订阅示例

```python
#!/usr/bin/env python3
"""
ros2_subscriber.py - 订阅 PX4 话题示例
"""
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

class DroneStateSubscriber(Node):
    def __init__(self):
        super().__init__('drone_state_subscriber')
        
        # 订阅位置
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            10
        )
        
        # 订阅姿态
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            10
        )
    
    def position_callback(self, msg):
        self.get_logger().info(
            f'位置: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}'
        )
    
    def attitude_callback(self, msg):
        # 四元数转欧拉角
        q = msg.q
        # ... 转换代码
        pass

def main():
    rclpy.init()
    node = DroneStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 10.5 已知问题：Isaac Sim 与 ROS2 Python 版本冲突

**问题**：
- Isaac Sim 使用 Python 3.11
- ROS2 JAZZY 系统安装使用 Python 3.12
- 导致 Pegasus 的 ROS2Backend 无法加载

**错误信息**：
```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```

**解决方案**：
1. 使用不依赖 ROS2Backend 的脚本（如 `sensor_test.py`）
2. 使用 Isaac Sim 内置的 OmniGraph ROS2 节点
3. 从独立的 Python 环境运行 ROS2 代码（不通过 Isaac Sim Python）

---

# 11. 常用命令速查表

## 11.1 系统命令

```bash
# GPU 状态
nvidia-smi

# 系统资源监控
htop

# 磁盘使用
df -h

# 内存使用
free -h

# CPU 信息
lscpu | head -20
```

## 11.2 仿真启动

```bash
# 完整仿真（推荐）
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py

# 带传感器的仿真
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/sensor_test.py

# 单独启动 Isaac Sim GUI
~/isaacsim/isaac-sim.sh

# 单独启动 PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl_default none

# QGroundControl
~/Downloads/QGroundControl.AppImage
```

## 11.3 进程管理

```bash
# 查看 PX4 进程
ps aux | grep px4

# 杀死 PX4 进程
pkill -9 px4

# 杀死 Isaac Sim
pkill -9 isaac

# 查看端口占用
ss -tuln | grep 14540
```

## 11.4 ROS2 命令

```bash
# 列出话题
ros2 topic list

# 查看话题数据
ros2 topic echo /topic_name

# 查看话题频率
ros2 topic hz /topic_name

# 列出节点
ros2 node list

# 查看服务
ros2 service list
```

## 11.5 PX4 命令 (pxh> 提示符)

```bash
# 状态查看
commander status        # 飞行器状态
ekf2 status            # EKF2 状态
sensor status          # 传感器状态

# 参数操作
param show EKF2*       # 查看 EKF2 参数
param set NAME VALUE   # 设置参数
param save             # 保存参数

# 飞行控制
commander arm          # 解锁
commander disarm       # 上锁
commander takeoff      # 起飞
commander land         # 降落

# 退出
shutdown
```

## 11.6 Git 命令

```bash
# 查看 PX4 版本
cd ~/PX4-Autopilot
git describe --tags

# 切换版本
git checkout v1.15.4
git submodule update --init --recursive

# 清理编译
make distclean
```

---

# 12. 故障排除

## 12.1 常见问题速查表

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| "Waiting for first heartbeat" 卡住 | 上次 PX4 进程未退出 | `pkill -9 px4` |
| Isaac Sim 黑屏/崩溃 | 着色器缓存损坏 | 清除缓存后重试 |
| PX4 编译失败 | 子模块未同步 | `make distclean` 后重新编译 |
| `ros2` 命令找不到 | 未加载环境 | `source /opt/ros/jazzy/setup.bash` |
| ROS2Backend 加载失败 | Python 版本冲突 | 使用不依赖 ROS2 的脚本 |
| GPU 内存不足 | 仿真过于复杂 | 降低分辨率或传感器数量 |

## 12.2 详细解决方案

### 问题1: "Waiting for first heartbeat" 一直显示

```bash
# 杀死残留进程
pkill -9 px4

# 检查端口占用
ss -tuln | grep 4560

# 如果端口被占用，找出并杀死进程
sudo fuser -k 4560/tcp

# 重新启动仿真
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py
```

### 问题2: Isaac Sim 启动失败或崩溃

```bash
# 清除所有缓存
rm -rf ~/.nvidia-omniverse/cache
rm -rf ~/.cache/ov
rm -rf ~/.cache/nvidia/GLCache
rm -rf ~/.nv/ComputeCache

# 检查 GPU 驱动
nvidia-smi

# 检查 GPU 温度（过热可能导致崩溃）
nvidia-smi -q -d TEMPERATURE

# 重启电脑
sudo reboot
```

### 问题3: PX4 编译失败

```bash
cd ~/PX4-Autopilot

# 完全清理
make distclean

# 重新同步子模块
git submodule sync --recursive
git submodule update --init --recursive --force

# 确认 GCC 版本
gcc --version  # 应该是 11.x

# 重新编译
make px4_sitl_default none
```

### 问题4: ROS2 相关错误

```bash
# 确保环境已加载
source /opt/ros/jazzy/setup.bash

# 检查 ROS2 是否正常
ros2 doctor

# 如果 px4_msgs 找不到
# 需要从源码编译 px4_msgs（高级）
```

### 问题5: GPU 性能警告

```bash
# 设置 CPU 为性能模式
sudo cpupower frequency-set -g performance

# 或者忽略警告（不影响功能）
```

---

# 13. 开发环境建议

## 13.1 推荐安装 VS Code

```bash
# 安装 VS Code
sudo snap install code --classic

# 安装推荐扩展
code --install-extension ms-python.python
code --install-extension ms-iot.vscode-ros
code --install-extension ms-vscode.cpptools
code --install-extension nvidia.nsight-vscode-edition
```

## 13.2 VS Code 配置

创建 `.vscode/settings.json`：

```json
{
    "python.defaultInterpreterPath": "~/isaacsim/python.sh",
    "python.analysis.extraPaths": [
        "~/isaacsim/exts",
        "~/PegasusSimulator/extensions"
    ],
    "files.associations": {
        "*.usd": "json",
        "*.usda": "plaintext"
    }
}
```

## 13.3 其他推荐工具

```bash
# 多窗口终端
sudo apt install terminator

# 交互式 Python
pip install ipython --break-system-packages

# 数据分析
pip install pandas matplotlib seaborn --break-system-packages

# 点云可视化
pip install open3d --break-system-packages

# 图像处理
pip install opencv-python --break-system-packages
```

## 13.4 推荐的项目结构

```
~/uav-sensor-spoofing/          # 你的研究代码仓库
├── README.md
├── requirements.txt
├── docs/
│   ├── installation.md         # 安装文档
│   └── manual.md              # 使用手册
├── src/
│   ├── spoofing/              # 欺骗模块
│   │   ├── __init__.py
│   │   ├── camera_spoof.py    # 相机欺骗
│   │   ├── lidar_spoof.py     # LiDAR 欺骗
│   │   └── gps_spoof.py       # GPS 欺骗
│   ├── perception/            # 感知模块
│   │   ├── __init__.py
│   │   ├── obstacle_detector.py
│   │   └── point_cloud_processor.py
│   └── utils/                 # 工具函数
│       ├── __init__.py
│       ├── logger.py
│       └── visualizer.py
├── experiments/               # 实验脚本
│   ├── baseline_test.py
│   ├── camera_attack_test.py
│   └── lidar_attack_test.py
├── config/
│   └── experiment_config.yaml
├── data/                      # 实验数据（.gitignore）
│   ├── baseline/
│   └── attacks/
└── results/                   # 结果分析
    └── plots/
```

---

# 14. 附录

## 14.1 重要文件路径

| 文件/目录 | 路径 | 用途 |
|-----------|------|------|
| Isaac Sim 主目录 | ~/isaacsim/ | 仿真平台 |
| Isaac Sim Python | ~/isaacsim/python.sh | Python 解释器 |
| Isaac Sim 启动脚本 | ~/isaacsim/isaac-sim.sh | GUI 启动 |
| PX4 主目录 | ~/PX4-Autopilot/ | 飞控软件 |
| EKF2 源码 | ~/PX4-Autopilot/src/modules/ekf2/ | 状态估计器 |
| Pegasus 主目录 | ~/PegasusSimulator/ | 仿真框架 |
| Pegasus 配置 | ~/PegasusSimulator/extensions/pegasus.simulator/config/configs.yaml | |
| MAVLink 后端 | ~/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py | 攻击注入点 |
| 相机传感器 | ~/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/graphical_sensors/monocular_camera.py | |
| LiDAR 传感器 | ~/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/graphical_sensors/lidar.py | |
| LiDAR 配置 | ~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/ | |
| ROS2 | /opt/ros/jazzy/ | 通信框架 |
| CUDA | /usr/local/cuda-13.0/ | GPU 计算 |
| 环境变量 | ~/.bashrc | 系统配置 |

## 14.2 ~/.bashrc 完整配置

```bash
# ========================================
# CUDA 13.0 环境变量
# ========================================
export PATH=/usr/local/cuda-13.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH

# ========================================
# ROS2 JAZZY 环境变量
# ========================================
source /opt/ros/jazzy/setup.bash

# ========================================
# Isaac Sim 环境变量
# ========================================
export ISAACSIM_PATH="${HOME}/isaacsim"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
alias isaac_run="${ISAACSIM_PATH}/isaac-sim.sh"
```

## 14.3 版本验证命令

| 组件 | 验证命令 | 预期输出 |
|------|----------|----------|
| Ubuntu | `lsb_release -a` | 24.04 LTS |
| NVIDIA Driver | `nvidia-smi` | 580.95.05 |
| CUDA | `nvcc --version` | 13.0 |
| GCC | `gcc --version` | 11.5.0 |
| ROS2 | `ros2 --version` | 或 `ros2 -h` |
| Isaac Sim | 查看窗口标题 | 5.1.0 |
| PX4 | `cd ~/PX4-Autopilot && git describe` | v1.15.4 |
| Pegasus | 查看 configs.yaml | 5.1.0 |

## 14.4 参考资料

| 资源 | 链接 |
|------|------|
| Isaac Sim 文档 | https://docs.isaacsim.omniverse.nvidia.com/ |
| PX4 文档 | https://docs.px4.io/ |
| ROS2 JAZZY 文档 | https://docs.ros.org/en/jazzy/ |
| Pegasus Simulator | https://pegasussimulator.github.io/PegasusSimulator/ |
| MAVSDK-Python | https://mavsdk.mavlink.io/main/en/python/ |
| MAVLink 协议 | https://mavlink.io/en/ |

## 14.5 联系信息

- **研究导师**: Dr. Shaocheng Luo
- **工作站用户**: rivery
- **研究方向**: 无人机感知传感器欺骗实验

---

# 文档更新记录

| 版本 | 日期 | 更新内容 |
|------|------|----------|
| 1.0 | 2026-01-17 | 初始版本 |
| 2.0 | 2026-01-18 | PX4 升级到 v1.15.4；添加感知传感器配置详解；添加感知传感器欺骗实验指南；更新系统架构图；添加已知问题和解决方案；完善代码示例 |

---

**文档结束**

*本文档由 Claude AI 协助生成*
