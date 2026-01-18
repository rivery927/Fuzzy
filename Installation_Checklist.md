# 无人机仿真平台安装清单

## 可复制安装指南 (Reproducible Installation Guide)

**目标配置**:
- Ubuntu 24.04 LTS
- NVIDIA Driver 580.x + CUDA 13.0
- ROS2 JAZZY
- Isaac Sim 5.1.0
- PX4-Autopilot v1.14.3
- Pegasus Simulator 5.1.0

**预计时间**: 2-3 小时（取决于网络速度）  
**预计磁盘空间**: 60-70 GB

---

# 前提条件

- 已安装 Ubuntu 24.04 LTS
- NVIDIA 显卡（RTX 30/40/50 系列）
- 稳定的网络连接
- 至少 100GB 可用磁盘空间

---

# 第一步：系统更新和基础工具

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
    lsb-release
```

**验证**:
```bash
gcc --version  # 应显示 gcc 13.x（Ubuntu 24.04 默认）
git --version
cmake --version
```

---

# 第二步：安装 NVIDIA 驱动

## 2.1 检查推荐驱动

```bash
ubuntu-drivers devices
```

## 2.2 安装推荐驱动（580-open）

```bash
sudo apt install -y nvidia-driver-580-open
```

> **注意**: 如果系统已预装驱动，此命令会显示"已是最新版本"

## 2.3 重启并验证

```bash
sudo reboot
```

重启后：
```bash
nvidia-smi
```

**预期输出**:
- Driver Version: 580.xx.xx
- CUDA Version: 13.0
- 显示你的 GPU 型号

---

# 第三步：安装 CUDA Toolkit 13.0

## 3.1 下载并安装 CUDA

```bash
cd ~/Downloads

# 下载 CUDA 13.0 (约 3.3GB)
wget https://developer.download.nvidia.com/compute/cuda/13.0.0/local_installers/cuda_13.0.0_555.42.02_linux.run

# 运行安装程序
sudo sh cuda_13.0.0_555.42.02_linux.run
```

**安装选项**:
1. 接受 EULA: 输入 `accept`
2. **取消勾选 Driver** (因为已经安装了)
3. 保持其他选项默认
4. 选择 Install

## 3.2 配置环境变量

```bash
echo '' >> ~/.bashrc
echo '# CUDA 13.0 环境变量' >> ~/.bashrc
echo 'export PATH=/usr/local/cuda-13.0/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc

source ~/.bashrc
```

## 3.3 验证安装

```bash
nvcc --version
```

**预期输出**: `Cuda compilation tools, release 13.0, V13.0.xxx`

---

# 第四步：安装 ROS2 JAZZY

## 4.1 设置软件源

```bash
# 添加 ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加软件源
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 4.2 安装 ROS2 JAZZY Desktop

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

> **注意**: 安装过程约 5-10 分钟，会下载 2-3 GB 的包

## 4.3 配置环境变量

```bash
echo '' >> ~/.bashrc
echo '# ROS2 JAZZY 环境变量' >> ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

source ~/.bashrc
```

## 4.4 验证安装

```bash
ros2 -h
```

**预期输出**: 显示 ROS2 命令列表

---

# 第五步：安装 Isaac Sim 5.1.0

## 5.1 创建目录并下载

```bash
mkdir -p ~/isaacsim
cd ~/isaacsim

# 下载 Isaac Sim (约 8GB，需要 5-15 分钟)
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

## 5.2 解压

```bash
cd ~/isaacsim
unzip isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

## 5.3 运行安装后脚本

```bash
cd ~/isaacsim
./post_install.sh
```

## 5.4 安装系统依赖

```bash
sudo apt install -y libfuse2t64 libxkbcommon0 libxcb-xinerama0 libxcb-cursor0
```

---

# 第六步：安装 GCC 11（Ubuntu 24.04 必需）

> **重要**: Isaac Sim 5.1.0 不兼容 GCC 12+，必须降级到 GCC 11

## 6.1 安装 GCC 11

```bash
sudo apt install -y gcc-11 g++-11
```

## 6.2 配置默认编译器

```bash
# 设置 GCC 11 为默认
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 110

# 设置 GCC 13 为备选（优先级较低）
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
```

## 6.3 验证

```bash
gcc --version
```

**预期输出**: `gcc (Ubuntu 11.x.x) 11.x.x`

---

# 第七步：配置 Isaac Sim 环境变量

```bash
echo '' >> ~/.bashrc
echo '# Isaac Sim 环境变量' >> ~/.bashrc
echo 'export ISAACSIM_PATH="${HOME}/isaacsim"' >> ~/.bashrc
echo 'export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"' >> ~/.bashrc
echo 'alias isaac_run="${ISAACSIM_PATH}/isaac-sim.sh"' >> ~/.bashrc

source ~/.bashrc
```

## 验证 Isaac Sim Python

```bash
$ISAACSIM_PYTHON -c "print('Isaac Sim Python OK')"
```

---

# 第八步：安装 PX4-Autopilot

## 8.1 安装 PX4 依赖

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

## 8.2 克隆 PX4 仓库

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

> **注意**: 这会下载约 500MB，需要几分钟

## 8.3 切换到 v1.14.3 版本

```bash
cd ~/PX4-Autopilot
git checkout -f v1.14.3
git submodule update --init --recursive --force
```

## 8.4 编译 PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl_default none
```

> **注意**: 第一次编译需要 3-10 分钟

**成功标志**: 看到 PX4 logo 和 `pxh>` 提示符

## 8.5 退出 PX4

在 `pxh>` 提示符下输入:
```
shutdown
```

---

# 第九步：安装 Pegasus Simulator

## 9.1 克隆仓库

```bash
cd ~
git clone https://github.com/PegasusSimulator/PegasusSimulator.git
```

## 9.2 安装 Pegasus

```bash
cd ~/PegasusSimulator/extensions
$ISAACSIM_PYTHON -m pip install --editable pegasus.simulator
```

---

# 第十步：验证完整安装

## 10.1 启动完整仿真

```bash
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py
```

> **注意**: 第一次启动 Isaac Sim 需要 5-15 分钟编译着色器

## 10.2 验证成功

**成功标志**:
1. Isaac Sim 窗口打开，显示 3D 场景和四旋翼无人机
2. 终端显示:
   - `Received first heartbeat`
   - `[commander] Ready for takeoff!`

## 10.3 如果看到 "Waiting for first heartbeat"

```bash
# 关闭 Isaac Sim (Ctrl+C)
# 杀死残留 PX4 进程
pkill -9 px4

# 重新启动
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py
```

---

# 安装后配置汇总

## ~/.bashrc 中应包含的内容

```bash
# CUDA 13.0 环境变量
export PATH=/usr/local/cuda-13.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH

# ROS2 JAZZY 环境变量
source /opt/ros/jazzy/setup.bash

# Isaac Sim 环境变量
export ISAACSIM_PATH="${HOME}/isaacsim"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
alias isaac_run="${ISAACSIM_PATH}/isaac-sim.sh"
```

---

# 安装目录结构

```
~/
├── isaacsim/                    # Isaac Sim 5.1.0
│   ├── isaac-sim.sh            # 主启动脚本
│   ├── python.sh               # Python 解释器
│   └── ...
├── PX4-Autopilot/               # PX4 飞控软件 (v1.14.3)
│   ├── src/
│   ├── build/
│   └── ...
└── PegasusSimulator/            # Pegasus 仿真框架
    ├── extensions/
    ├── examples/
    └── ...
```

---

# 常见问题

## Q1: Isaac Sim 启动时黑屏或崩溃
```bash
# 清除缓存
rm -rf ~/.nvidia-omniverse/cache
# 重启电脑后再试
sudo reboot
```

## Q2: PX4 编译失败
```bash
cd ~/PX4-Autopilot
make clean
pip install kconfiglib future --break-system-packages
make px4_sitl_default none
```

## Q3: ROS2 命令找不到
```bash
source /opt/ros/jazzy/setup.bash
```

## Q4: "Waiting for first heartbeat" 一直显示
```bash
pkill -9 px4
# 重新启动仿真
```

---

# 快速启动命令

```bash
# 启动完整仿真
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/1_px4_single_vehicle.py

# 单独启动 PX4
cd ~/PX4-Autopilot
make px4_sitl_default none

# 查看 GPU 状态
nvidia-smi

# 查看 ROS2 话题
ros2 topic list
```

---

# 版本信息

| 组件 | 版本 | 验证命令 |
|------|------|----------|
| Ubuntu | 24.04 LTS | `lsb_release -a` |
| NVIDIA Driver | 580.x | `nvidia-smi` |
| CUDA | 13.0 | `nvcc --version` |
| GCC | 11.x | `gcc --version` |
| ROS2 | JAZZY | `ros2 -h` |
| Isaac Sim | 5.1.0 | 查看启动窗口标题 |
| PX4 | v1.14.3 | `cd ~/PX4-Autopilot && git describe` |
| Pegasus | 5.1.0 | 查看 configs.yaml |

---

**文档创建日期**: 2026-01-17  
**测试环境**: Alienware 16 Area-51 (RTX 5080)
