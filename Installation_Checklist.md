# 无人机仿真平台安装清单 (完整版)

## UAV Simulation Platform Installation Guide

**版本**: 2.0  
**最后更新**: 2026-01-18  
**测试环境**: Alienware 16 Area-51 (Intel Core Ultra 9 275HX + RTX 5080)

---

## 目标配置

| 组件 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 24.04 LTS | 操作系统基础 |
| NVIDIA Driver | 580.x | GPU 驱动 |
| CUDA Toolkit | 13.0 | GPU 计算库 |
| GCC/G++ | 11.x | 编译器（Isaac Sim 兼容性要求） |
| ROS2 | JAZZY | 机器人通信框架 |
| Isaac Sim | 5.1.0 | NVIDIA 仿真平台 |
| PX4-Autopilot | **v1.15.4** | 无人机飞控软件 |
| Pegasus Simulator | 5.1.0 | 无人机仿真框架 |

**预计时间**: 2-3 小时（取决于网络速度）  
**预计磁盘空间**: 70-80 GB

---

## 前提条件

- ✅ 已安装 Ubuntu 24.04 LTS
- ✅ NVIDIA 显卡（RTX 30/40/50 系列，推荐 RTX 40/50 系列）
- ✅ 稳定的网络连接（需下载约 20GB 数据）
- ✅ 至少 100GB 可用磁盘空间
- ✅ 至少 16GB 系统内存（推荐 32GB）

---

# 第一部分：系统基础配置

## 第一步：系统更新和基础工具

```bash
# 更新软件包列表和系统
sudo apt update && sudo apt upgrade -y

# 安装基础开发工具
sudo apt install -y \
    build-essential \
    git \
    curl \
    wget \
    cmake \
    python3-pip \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    ninja-build \
    zip \
    unzip
```

**验证**:
```bash
gcc --version  # 应显示 gcc 13.x（Ubuntu 24.04 默认）
git --version
cmake --version
```

---

## 第二步：安装 NVIDIA 驱动

### 2.1 检查推荐驱动

```bash
ubuntu-drivers devices
```

**预期输出示例**:
```
== /sys/devices/pci0000:00/0000:00:06.0/0000:02:00.0 ==
vendor   : NVIDIA Corporation
driver   : nvidia-driver-580-open - distro non-free recommended
```

### 2.2 安装推荐驱动

```bash
sudo apt install -y nvidia-driver-580-open
```

> **注意**: 
> - RTX 50 系列（Blackwell 架构）需要 570+ 驱动
> - RTX 40 系列需要 525+ 驱动
> - 如果系统已预装驱动，此命令会显示"已是最新版本"

### 2.3 重启并验证

```bash
sudo reboot
```

重启后验证：
```bash
nvidia-smi
```

**预期输出**:
```
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 580.95.05              Driver Version: 580.95.05      CUDA Version: 13.0     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 5080 ...    Off | 00000000:02:00.0  On |                  N/A |
| N/A   45C    P8              15W / 175W |     502MiB / 16303MiB |      2%      Default |
+-----------------------------------------+------------------------+----------------------+
```

---

## 第三步：安装 CUDA Toolkit 13.0

### 3.1 下载并安装 CUDA

```bash
cd ~/Downloads

# 下载 CUDA 13.0 (约 3.3GB)
wget https://developer.download.nvidia.com/compute/cuda/13.0.0/local_installers/cuda_13.0.0_555.42.02_linux.run

# 运行安装程序
sudo sh cuda_13.0.0_555.42.02_linux.run
```

**安装选项**:
1. 接受 EULA: 输入 `accept`
2. ⚠️ **取消勾选 Driver**（因为已经安装了）
3. 保持其他选项默认（Toolkit、Samples 等）
4. 选择 Install

### 3.2 配置环境变量

```bash
echo '' >> ~/.bashrc
echo '# CUDA 13.0 环境变量' >> ~/.bashrc
echo 'export PATH=/usr/local/cuda-13.0/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc

source ~/.bashrc
```

### 3.3 验证安装

```bash
nvcc --version
```

**预期输出**: 
```
nvcc: NVIDIA (R) Cuda compiler driver
Cuda compilation tools, release 13.0, V13.0.xxx
```

---

## 第四步：安装 GCC 11（Ubuntu 24.04 必需）

> ⚠️ **重要**: Isaac Sim 5.1.0 使用 GCC 11 编译，与 GCC 12+ 不兼容。必须降级默认编译器。

### 4.1 安装 GCC 11

```bash
sudo apt install -y gcc-11 g++-11
```

### 4.2 配置默认编译器

```bash
# 设置 GCC 11 为默认（优先级 110）
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 110

# 设置 GCC 13 为备选（优先级 100，较低）
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
```

### 4.3 验证

```bash
gcc --version
g++ --version
```

**预期输出**: `gcc (Ubuntu 11.x.x) 11.x.x`

> **提示**: 如需切换回 GCC 13，运行 `sudo update-alternatives --config gcc`

---

## 第五步：安装 ROS2 JAZZY

### 5.1 设置软件源

```bash
# 添加 ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加软件源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 5.2 安装 ROS2 JAZZY Desktop

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

> **注意**: 安装过程约 5-10 分钟，会下载 2-3 GB 的包

### 5.3 配置环境变量

```bash
echo '' >> ~/.bashrc
echo '# ROS2 JAZZY 环境变量' >> ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

source ~/.bashrc
```

### 5.4 验证安装

```bash
ros2 -h
```

**预期输出**: 显示 ROS2 命令列表（action, bag, component, daemon, doctor 等）

---

# 第二部分：仿真平台安装

## 第六步：安装 Isaac Sim 5.1.0

### 6.1 创建目录并下载

```bash
mkdir -p ~/isaacsim
cd ~/isaacsim

# 下载 Isaac Sim (约 8GB，需要 5-15 分钟)
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

### 6.2 解压

```bash
cd ~/isaacsim
unzip isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

> **注意**: 解压后约占用 50GB 磁盘空间

### 6.3 运行安装后脚本

```bash
cd ~/isaacsim
./post_install.sh
```

### 6.4 安装系统依赖

```bash
sudo apt install -y libfuse2t64 libxkbcommon0 libxcb-xinerama0 libxcb-cursor0
```

### 6.5 配置环境变量

```bash
echo '' >> ~/.bashrc
echo '# Isaac Sim 环境变量' >> ~/.bashrc
echo 'export ISAACSIM_PATH="${HOME}/isaacsim"' >> ~/.bashrc
echo 'export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"' >> ~/.bashrc
echo 'alias isaac_run="${ISAACSIM_PATH}/isaac-sim.sh"' >> ~/.bashrc

source ~/.bashrc
```

### 6.6 验证 Isaac Sim

```bash
$ISAACSIM_PYTHON -c "print('Isaac Sim Python OK')"
```

> **注意**: 第一次启动 Isaac Sim 需要 5-15 分钟编译着色器（shader warmup）

---

## 第七步：安装 PX4-Autopilot v1.15.4

### 7.1 安装 PX4 依赖

```bash
sudo apt install -y \
    astyle \
    ninja-build \
    libimage-exiftool-perl \
    python3-jinja2 \
    python3-pip \
    python3-jsonschema \
    python3-numpy \
    python3-toml \
    python3-empy \
    python3-packaging \
    libxml2-dev \
    libxslt1-dev \
    zip \
    openjdk-21-jdk

# 安装 Python 依赖
pip install pyros-genmsg kconfiglib future --break-system-packages
```

### 7.2 克隆 PX4 仓库

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

> **注意**: 这会下载约 500MB，需要几分钟

### 7.3 切换到 v1.15.4 版本

```bash
cd ~/PX4-Autopilot

# 获取所有标签
git fetch --all --tags

# 切换到 v1.15.4
git checkout v1.15.4

# 更新所有子模块（重要！）
git submodule update --init --recursive --force
```

### 7.4 编译 PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl_default none
```

> **注意**: 第一次编译需要 5-10 分钟

**成功标志**: 
```
______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.
...
pxh> 
```

### 7.5 退出 PX4

在 `pxh>` 提示符下输入:
```
shutdown
```

### 7.6 验证版本

```bash
cd ~/PX4-Autopilot
git describe --tags
```

**预期输出**: `v1.15.4`

---

## 第八步：安装 Pegasus Simulator

### 8.1 克隆仓库

```bash
cd ~
git clone https://github.com/PegasusSimulator/PegasusSimulator.git
```

### 8.2 安装 Pegasus

```bash
cd ~/PegasusSimulator/extensions
$ISAACSIM_PYTHON -m pip install --editable pegasus.simulator
```

---

## 第九步：验证完整安装

### 9.1 启动完整仿真

```bash
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py
```

### 9.2 验证成功

**成功标志**:
1. Isaac Sim 窗口打开，显示 3D 场景和四旋翼无人机
2. 终端显示:
   - `Received first heartbeat`
   - `[commander] Ready for takeoff!`

### 9.3 故障排除

如果看到 "Waiting for first heartbeat" 一直显示：

```bash
# 关闭 Isaac Sim (Ctrl+C)
# 杀死残留 PX4 进程
pkill -9 px4

# 重新启动
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py
```

---

# 第三部分：感知传感器配置

## 第十步：配置相机和 LiDAR

博士后要求配置以下感知传感器用于欺骗实验：
- 相机（第一人称视角）
- LiDAR
- 毫米波雷达（可选）

### 10.1 传感器测试脚本

已创建测试脚本 `~/PegasusSimulator/examples/sensor_test.py`，包含：

| 传感器 | 配置 | 说明 |
|--------|------|------|
| 前置相机 | 640x480, 30Hz, RGB+Depth | 位于无人机前方 15cm |
| LiDAR | Example_Rotary, 10Hz | 位于无人机顶部 5cm |
| 障碍物 | 3个彩色方块 | 红、蓝、绿，用于感知测试 |

### 10.2 运行传感器测试

```bash
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/sensor_test.py
```

### 10.3 可用的 LiDAR 配置

Isaac Sim 支持多种 LiDAR 型号：

| 品牌 | 型号示例 |
|------|----------|
| Velodyne | VLS128 |
| Ouster | OS0, OS1, OS2 (多种配置) |
| HESAI | 多种型号 |
| SICK | 多种型号 |
| SLAMTEC | 多种型号 |
| ZVISION | 多种型号 |

配置文件位置：`~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/`

---

# 第四部分：已知问题与解决方案

## 问题 1: Isaac Sim 与 ROS2 Python 版本冲突

**问题描述**:
- Isaac Sim 使用 Python 3.11
- ROS2 JAZZY 系统安装使用 Python 3.12
- 导致 Pegasus 的 `ROS2Backend` 无法加载

**错误信息**:
```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '...cpython-311-x86_64-linux-gnu.so' isn't present on the system
```

**解决方案**:
1. 使用不依赖 ROS2Backend 的脚本（如 `sensor_test.py`）
2. 或使用 Isaac Sim 内置的 OmniGraph ROS2 节点发布传感器数据

**Isaac Sim 内置 ROS2 环境设置**:
```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib
```

## 问题 2: PX4 编译失败 (microcdr 版本不匹配)

**错误信息**:
```
Could not find a configuration file for package "microcdr" that exactly matches requested version "2.0.1"
```

**解决方案**:
```bash
cd ~/PX4-Autopilot
make distclean
git submodule sync --recursive
git submodule update --init --recursive --force
make px4_sitl_default none
```

## 问题 3: Isaac Sim 启动时黑屏或崩溃

**解决方案**:
```bash
# 清除着色器缓存
rm -rf ~/.nvidia-omniverse/cache
rm -rf ~/.cache/ov
rm -rf ~/.cache/nvidia/GLCache

# 重启电脑后再试
sudo reboot
```

## 问题 4: "Waiting for first heartbeat" 持续显示

**原因**: 上次运行的 PX4 进程未完全退出

**解决方案**:
```bash
pkill -9 px4
# 然后重新启动仿真
```

## 问题 5: GPU 性能警告

**警告信息**:
```
CPU performance profile is set to powersave
PCIe link width current (8) and maximum (16) don't match
```

**解决方案**（可选，不影响功能）:
```bash
# 设置 CPU 为性能模式
sudo cpupower frequency-set -g performance
```

---

# 第五部分：环境配置汇总

## ~/.bashrc 完整配置

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

---

# 第六部分：目录结构

```
~/
├── isaacsim/                          # Isaac Sim 5.1.0 (~50GB)
│   ├── isaac-sim.sh                   # GUI 启动脚本
│   ├── python.sh                      # Python 解释器
│   ├── exts/                          # 扩展模块
│   │   ├── isaacsim.ros2.bridge/      # ROS2 桥接
│   │   │   ├── jazzy/                 # JAZZY 支持
│   │   │   └── humble/                # HUMBLE 支持
│   │   └── isaacsim.sensors.rtx/      # RTX 传感器
│   │       └── data/lidar_configs/    # LiDAR 配置文件
│   └── ...
│
├── PX4-Autopilot/                     # PX4 飞控 v1.15.4 (~2GB)
│   ├── src/                           # 源代码
│   │   └── modules/
│   │       └── ekf2/                  # EKF2 状态估计器（攻击目标）
│   ├── build/                         # 编译输出
│   └── ROMFS/                         # 配置文件
│
├── PegasusSimulator/                  # Pegasus 仿真框架 (~100MB)
│   ├── extensions/
│   │   └── pegasus.simulator/
│   │       └── pegasus/simulator/
│   │           └── logic/
│   │               ├── backends/
│   │               │   └── px4_mavlink_backend.py  # MAVLink 后端（攻击注入点）
│   │               └── graphical_sensors/
│   │                   ├── monocular_camera.py     # 相机传感器
│   │                   └── lidar.py                # LiDAR 传感器
│   └── examples/
│       ├── 1_px4_single_vehicle.py    # 基础示例
│       ├── 8_camera_vehicle.py        # 相机示例
│       ├── sensor_test.py             # 传感器测试（我们创建的）
│       └── sensor_ros2.py             # ROS2 传感器（我们创建的）
│
└── start_sensor_sim.sh                # 传感器仿真启动脚本
```

---

# 第七部分：快速参考

## 常用命令

```bash
# ========================================
# 启动仿真
# ========================================

# 基础仿真（无人机 + PX4）
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py

# 带传感器的仿真（相机 + LiDAR）
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/sensor_test.py

# ========================================
# PX4 操作
# ========================================

# 单独启动 PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl_default none

# 在 pxh> 提示符下的常用命令
commander takeoff          # 起飞
commander land             # 降落
param show SYS_AUTOSTART   # 查看参数
ekf2 status                # EKF2 状态
shutdown                   # 退出

# 杀死残留 PX4 进程
pkill -9 px4

# ========================================
# 系统检查
# ========================================

# GPU 状态
nvidia-smi

# CUDA 版本
nvcc --version

# GCC 版本
gcc --version

# ROS2 话题列表
ros2 topic list

# PX4 版本
cd ~/PX4-Autopilot && git describe --tags
```

## 版本验证清单

| 组件 | 验证命令 | 预期输出 |
|------|----------|----------|
| Ubuntu | `lsb_release -a` | 24.04 LTS |
| NVIDIA Driver | `nvidia-smi` | 580.x |
| CUDA | `nvcc --version` | 13.0 |
| GCC | `gcc --version` | 11.x |
| ROS2 | `ros2 -h` | 命令列表 |
| Isaac Sim | 查看窗口标题 | 5.1.0 |
| PX4 | `cd ~/PX4-Autopilot && git describe` | v1.15.4 |
| Pegasus | 查看 configs.yaml | 5.1.0 |

---

# 第八部分：研究方向说明

## 博士后研究要求

根据博士后 Dr. Shaocheng Luo 的邮件，研究重点是：

> "We will need to inject spoofed/fake objects/points into these perceptual sensors. That is our research focus."

### 研究类型：感知传感器欺骗

| 传感器 | 攻击方式 | 目标 |
|--------|----------|------|
| 相机 | 注入虚假物体到图像 | 欺骗视觉检测系统 |
| LiDAR | 注入虚假点到点云 | 欺骗 3D 感知系统 |
| 毫米波雷达 | 注入虚假目标 | 欺骗测距/测速系统 |

### 与导航传感器欺骗的区别

| 类型 | 目标传感器 | 攻击效果 |
|------|------------|----------|
| **感知传感器欺骗**（研究重点） | 相机、LiDAR、雷达 | 让无人机"看到"虚假物体或"看不到"真实物体 |
| 导航传感器欺骗 | GPS、IMU、气压计、磁力计 | 让无人机飞错位置或姿态错误 |

---

# 附录

## A. 系统架构图

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
│  │   │  (Iris)        │      │  ★ 传感器数据注入点 ★           │     │   │
│  │   └────────┬────────┘      └──────────────┬──────────────────┘     │   │
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
│  │   │  ★ 攻击目标 ★  │      └──────────────────────────────────┘     │   │
│  │   └─────────────────┘                                               │   │
│  │                                                                     │   │
│  │   uXRCE-DDS (UDP 8888)                                              │   │
│  │            │                                                        │   │
│  └────────────┼────────────────────────────────────────────────────────┘   │
│               │                                                            │
│               ▼                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      ROS2 JAZZY                                      │   │
│  │                                                                     │   │
│  │   话题:                                                              │   │
│  │   /fmu/out/vehicle_local_position   (位置估计)                       │   │
│  │   /fmu/out/vehicle_attitude         (姿态估计)                       │   │
│  │   /fmu/out/sensor_combined          (传感器数据)                     │   │
│  │                                                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## B. 数据流频率

| 数据流 | 频率 | 说明 |
|--------|------|------|
| 物理仿真 | 250 Hz | Isaac Sim PhysX |
| IMU 数据 | 250 Hz | HIL_SENSOR |
| GPS 数据 | 10 Hz | HIL_GPS |
| 相机图像 | 30 Hz | 可配置 |
| LiDAR 点云 | 10 Hz | 可配置 |
| EKF2 输出 | 250 Hz | 状态估计 |

## C. 重要文件路径

| 文件 | 路径 | 用途 |
|------|------|------|
| Isaac Sim Python | `~/isaacsim/python.sh` | 运行仿真脚本 |
| PX4 目录 | `~/PX4-Autopilot/` | 飞控源码 |
| Pegasus 配置 | `~/PegasusSimulator/extensions/pegasus.simulator/config/configs.yaml` | 仿真配置 |
| MAVLink 后端 | `~/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/backends/px4_mavlink_backend.py` | 传感器数据注入点 |
| LiDAR 配置 | `~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/` | LiDAR 型号配置 |

---

**文档版本历史**:
| 版本 | 日期 | 更新内容 |
|------|------|----------|
| 1.0 | 2026-01-17 | 初始版本 |
| 2.0 | 2026-01-18 | PX4 升级到 v1.15.4；添加传感器配置；添加已知问题；完善系统架构说明 |
