# 无人机仿真平台 - 完整文件结构说明

**创建日期**: 2026-01-18  
**系统用户**: rivery  
**工作站**: Alienware 16 Area-51

---

## 📊 磁盘使用汇总

| 组件 | 大小 | 说明 |
|------|------|------|
| Isaac Sim | 27 GB | 3D 物理仿真平台 |
| PX4-Autopilot | 2.9 GB | 无人机飞控软件 |
| Pegasus Simulator | 230 MB | 无人机仿真框架 |
| CUDA 13.0 | ~5 GB | GPU 计算库 |
| ROS2 JAZZY | ~3 GB | 机器人通信框架 |
| **总计** | **~40 GB** | |

---

## 📁 完整文件结构

```
/home/rivery/                                    # 用户主目录
│
├── isaacsim/                                    # 【Isaac Sim 5.1.0】3D物理仿真平台 (27GB)
│   │
│   ├── isaac-sim.sh                             # Isaac Sim GUI 启动脚本
│   ├── python.sh                                # ★ Isaac Sim Python 解释器 (用 $ISAACSIM_PYTHON 调用)
│   ├── clear_caches.sh                          # 清除缓存脚本
│   ├── post_install.sh                          # 安装后配置脚本
│   ├── isaac-sim.streaming.sh                   # 流式传输启动脚本
│   ├── isaac-sim.selector.sh                    # 版本选择器
│   ├── isaac-sim.compatibility_check.sh         # 兼容性检查
│   ├── isaac-sim.fabric.sh                      # Fabric 模式启动
│   ├── isaac-sim.xr.vr.sh                       # VR 模式启动
│   ├── isaac-sim.action_and_event_data_generation.sh  # 数据生成
│   │
│   ├── environment.yml                          # Conda 环境配置
│   ├── config/                                  # 配置文件目录
│   ├── data/                                    # 数据文件目录
│   ├── docs/                                    # 文档目录
│   │
│   ├── apps/                                    # 应用配置文件 (.kit)
│   │
│   ├── exts/                                    # ★ 扩展模块目录 (核心功能)
│   │   │
│   │   ├── isaacsim.ros2.bridge/                # 【ROS2 桥接扩展】
│   │   │   ├── jazzy/                           #   JAZZY 版本支持
│   │   │   │   ├── lib/                         #     ROS2 库文件
│   │   │   │   └── rclpy/                       #     Python ROS2 绑定
│   │   │   └── humble/                          #   HUMBLE 版本支持
│   │   │
│   │   ├── isaacsim.ros2.sim_control/           # ROS2 仿真控制扩展
│   │   ├── isaacsim.ros2.tf_viewer/             # ROS2 TF 可视化
│   │   ├── isaacsim.ros2.urdf/                  # ROS2 URDF 导入
│   │   │
│   │   ├── isaacsim.sensors.camera/             # 【相机传感器扩展】
│   │   ├── isaacsim.sensors.camera.ui/          #   相机 UI 界面
│   │   ├── isaacsim.sensors.physics/            # 【物理传感器扩展】(IMU/接触等)
│   │   ├── isaacsim.sensors.physics.ui/         #   物理传感器 UI
│   │   ├── isaacsim.sensors.physics.examples/   #   传感器示例
│   │   ├── isaacsim.gui.sensors.icon/           #   传感器图标
│   │   │
│   │   └── isaacsim.sensors.rtx/                # 【RTX 传感器扩展】(LiDAR/雷达)
│   │       └── data/
│   │           └── lidar_configs/               # ★ LiDAR 配置文件
│   │               ├── Velodyne/                #   Velodyne LiDAR (VLS128)
│   │               ├── Ouster/                  #   Ouster LiDAR (OS0/OS1/OS2)
│   │               │   ├── OS0/                 #     近距离高密度
│   │               │   ├── OS1/                 #     中距离通用 ★推荐
│   │               │   └── OS2/                 #     远距离
│   │               ├── HESAI/                   #   禾赛 LiDAR
│   │               ├── SICK/                    #   SICK LiDAR
│   │               ├── SLAMTEC/                 #   思岚 LiDAR
│   │               ├── ZVISION/                 #   智驾 LiDAR
│   │               ├── NVIDIA/                  #   NVIDIA 示例配置
│   │               └── README.md                #   配置说明
│   │
│   ├── extscache/                               # 扩展缓存
│   ├── extsDeprecated/                          # 已弃用扩展
│   ├── extsUser/                                # 用户自定义扩展
│   ├── extension_examples/                      # 扩展示例
│   │
│   ├── kit/                                     # Omniverse Kit 核心
│   │   ├── python/                              #   Python 3.11 环境
│   │   └── logs/                                #   日志文件
│   │
│   ├── isaac-sim-5.1.0.zip                      # 原始压缩包 (可删除节省空间)
│   └── isaac-sim-standalone-5.1.0-linux-x86_64.zip  # 下载的安装包 (可删除)
│
│
├── PX4-Autopilot/                               # 【PX4 飞控软件 v1.15.4】(2.9GB)
│   │
│   ├── Makefile                                 # 编译入口
│   ├── CMakeLists.txt                           # CMake 配置
│   ├── Kconfig                                  # 内核配置
│   ├── LICENSE                                  # MIT 许可证
│   ├── README.md                                # 项目说明
│   ├── CONTRIBUTING.md                          # 贡献指南
│   ├── CODE_OF_CONDUCT.md                       # 行为准则
│   │
│   ├── src/                                     # ★ 源代码目录
│   │   │
│   │   └── modules/                             # ★ 核心模块
│   │       │
│   │       ├── ekf2/                            # ★★★【EKF2 状态估计器】导航攻击核心目标
│   │       │   ├── EKF2.cpp                     #   主程序
│   │       │   ├── EKF2.hpp                     #   头文件
│   │       │   ├── EKF/                         #   EKF 算法实现
│   │       │   └── test/                        #   测试代码
│   │       │
│   │       ├── commander/                       # 【指挥官模块】飞行模式管理
│   │       ├── flight_mode_manager/             # 飞行模式管理器
│   │       │
│   │       ├── mc_pos_control/                  # 【多旋翼位置控制器】
│   │       ├── mc_att_control/                  # 【多旋翼姿态控制器】
│   │       ├── mc_rate_control/                 # 多旋翼角速度控制器
│   │       │
│   │       ├── fw_pos_control/                  # 固定翼位置控制
│   │       ├── fw_att_control/                  # 固定翼姿态控制
│   │       ├── fw_rate_control/                 # 固定翼角速度控制
│   │       │
│   │       ├── control_allocator/               # 控制分配器
│   │       ├── battery_status/                  # 电池状态监控
│   │       ├── camera_feedback/                 # 相机反馈
│   │       ├── dataman/                         # 数据管理
│   │       ├── events/                          # 事件系统
│   │       ├── gimbal/                          # 云台控制
│   │       ├── gyro_calibration/                # 陀螺仪校准
│   │       ├── gyro_fft/                        # 陀螺仪 FFT 分析
│   │       ├── airspeed_selector/               # 空速选择器
│   │       ├── attitude_estimator_q/            # 四元数姿态估计
│   │       └── ...                              # 其他模块
│   │
│   ├── build/                                   # 编译输出目录
│   │   └── px4_sitl_default/                    #   SITL 编译产物
│   │
│   ├── boards/                                  # 硬件板配置
│   ├── cmake/                                   # CMake 模块
│   ├── Documentation/                           # 文档
│   ├── integrationtests/                        # 集成测试
│   ├── launch/                                  # ROS2 启动文件
│   ├── msg/                                     # uORB 消息定义
│   ├── platforms/                               # 平台适配代码
│   ├── ROMFS/                                   # 配置和启动脚本
│   │   └── px4fmu_common/
│   │       └── init.d-posix/                    #   SITL 启动配置
│   ├── Tools/                                   # 工具脚本
│   └── test/                                    # 测试代码
│
│
├── PegasusSimulator/                            # 【Pegasus 仿真框架 5.1.0】(230MB)
│   │
│   ├── README.md                                # 项目说明
│   ├── LICENSE                                  # BSD-3 许可证
│   ├── link_app.sh                              # Linux 链接脚本
│   ├── link_app.bat                             # Windows 链接脚本
│   │
│   ├── docs/                                    # 文档目录
│   ├── tools/                                   # 工具脚本
│   │
│   ├── examples/                                # ★ 示例脚本目录
│   │   │
│   │   ├── 0_template_app.py                    # 模板应用
│   │   ├── 1_px4_single_vehicle.py              # ★ PX4 单机仿真 (常用)
│   │   ├── 2_px4_multi_vehicle.py               # PX4 多机仿真
│   │   ├── 3_ros2_single_vehicle.py             # ROS2 单机仿真
│   │   ├── 4_python_single_vehicle.py           # Python 控制单机
│   │   ├── 5_python_multi_vehicle.py            # Python 控制多机
│   │   ├── 6_paper_results.py                   # 论文结果复现
│   │   ├── 8_camera_vehicle.py                  # 相机示例 (官方)
│   │   ├── 9_people.py                          # 行人仿真
│   │   ├── 10_graphs.py                         # 图形化示例
│   │   ├── 11_ardupilot_multi_vehicle.py        # ArduPilot 多机
│   │   │
│   │   ├── sensor_test.py                       # ★★ 我创建：传感器测试 (推荐使用)
│   │   ├── sensor_ros2.py                       # ★ 我创建：ROS2 传感器版本
│   │   │
│   │   ├── results/                             # 结果输出目录
│   │   ├── trajectories/                        # 轨迹文件
│   │   └── utils/                               # 工具函数
│   │
│   └── extensions/                              # 扩展目录
│       │
│       └── pegasus.simulator/                   # Pegasus 核心扩展
│           │
│           ├── config/
│           │   └── configs.yaml                 # ★ Pegasus 配置文件
│           │
│           └── pegasus/simulator/
│               │
│               ├── logic/                       # 核心逻辑
│               │   │
│               │   ├── backends/                # ★★ 通信后端 (攻击注入点)
│               │   │   ├── __init__.py
│               │   │   ├── backend.py           #   后端基类
│               │   │   ├── px4_mavlink_backend.py   # ★★★ PX4 MAVLink 后端 (传感器数据注入点)
│               │   │   ├── ardupilot_mavlink_backend.py  # ArduPilot 后端
│               │   │   ├── ros2_backend.py      #   ROS2 后端 (有 Python 版本冲突)
│               │   │   └── tools/               #   工具函数
│               │   │
│               │   ├── graphical_sensors/       # ★ 图形传感器 (感知攻击目标)
│               │   │   ├── __init__.py
│               │   │   ├── graphical_sensor.py  #   传感器基类
│               │   │   ├── monocular_camera.py  # ★ 单目相机实现
│               │   │   └── lidar.py             # ★ LiDAR 实现
│               │   │
│               │   ├── sensors/                 # 导航传感器 (导航攻击目标)
│               │   │   ├── imu.py               #   IMU 传感器
│               │   │   ├── gps.py               #   GPS 传感器
│               │   │   ├── barometer.py         #   气压计
│               │   │   └── magnetometer.py      #   磁力计
│               │   │
│               │   ├── vehicles/                # 载具定义
│               │   │   └── multirotor.py        #   多旋翼配置
│               │   │
│               │   └── interface/               # 接口
│               │       └── pegasus_interface.py #   Pegasus 主接口
│               │
│               ├── params/                      # 参数定义
│               └── ui/                          # UI 界面
│
│
├── start_sensor_sim.sh                          # ★ 我创建：传感器仿真启动脚本
│                                                #   设置 Isaac Sim 内置 ROS2 环境
│
├── .bashrc                                      # ★ Shell 配置文件 (环境变量)
│
├── .nvidia-omniverse/                           # NVIDIA Omniverse 配置和缓存
│   └── cache/                                   #   着色器缓存 (首次启动编译)
│
├── .nv/                                         # NVIDIA 缓存
├── .cache/                                      # 系统缓存
├── .config/                                     # 应用配置
├── .local/                                      # 本地数据
│
├── cuda-keyring_1.1-1_all.deb                   # CUDA 密钥包 (可删除)
│
├── snap/                                        # Snap 应用数据
│
├── 下载/                                        # Downloads 目录
├── 图片/                                        # Pictures 目录  
├── 文档/                                        # Documents 目录
├── 桌面/                                        # Desktop 目录
├── 公共/                                        # Public 目录
├── 模板/                                        # Templates 目录
├── 视频/                                        # Videos 目录
└── 音乐/                                        # Music 目录


/usr/local/                                      # 系统本地安装
│
├── cuda -> cuda-13.0/                           # CUDA 符号链接
├── cuda-13/                                     # CUDA 13 链接
└── cuda-13.0/                                   # 【CUDA 13.0 Toolkit】
    ├── bin/
    │   └── nvcc                                 # CUDA 编译器
    ├── lib64/                                   # CUDA 库文件
    ├── include/                                 # CUDA 头文件
    └── ...


/opt/ros/                                        # ROS 安装目录
│
└── jazzy/                                       # 【ROS2 JAZZY】
    ├── setup.bash                               # ★ 环境配置脚本
    ├── lib/
    │   └── python3.12/                          # Python 3.12 绑定
    └── share/                                   # ROS2 包


/usr/bin/                                        # 系统可执行文件
│
├── gcc -> gcc-11                                # GCC 默认版本 (11)
├── gcc-11                                       # 【GCC 11.5.0】★ 默认编译器
├── gcc-13                                       # GCC 13 (备选)
├── g++ -> g++-11                                # G++ 默认版本
├── g++-11                                       # G++ 11
└── g++-13                                       # G++ 13
```

---

## ⭐ 关键文件快速索引

### 日常使用

| 文件 | 路径 | 用途 |
|------|------|------|
| Isaac Sim Python | `~/isaacsim/python.sh` | 运行仿真脚本，用 `$ISAACSIM_PYTHON` |
| 基础仿真 | `~/PegasusSimulator/examples/1_px4_single_vehicle.py` | 最常用的仿真启动 |
| 传感器仿真 | `~/PegasusSimulator/examples/sensor_test.py` | 带相机+LiDAR 的仿真 |
| 启动脚本 | `~/start_sensor_sim.sh` | 设置 ROS2 环境并启动 |

### 研究开发

| 文件 | 路径 | 用途 |
|------|------|------|
| MAVLink 后端 | `~/PegasusSimulator/.../backends/px4_mavlink_backend.py` | ★ 传感器数据注入点 |
| 相机传感器 | `~/PegasusSimulator/.../graphical_sensors/monocular_camera.py` | 相机配置修改 |
| LiDAR 传感器 | `~/PegasusSimulator/.../graphical_sensors/lidar.py` | LiDAR 配置修改 |
| EKF2 源码 | `~/PX4-Autopilot/src/modules/ekf2/` | 状态估计器源码 |
| LiDAR 配置 | `~/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs/` | LiDAR 型号配置 |

### 配置文件

| 文件 | 路径 | 用途 |
|------|------|------|
| 环境变量 | `~/.bashrc` | CUDA/ROS2/Isaac Sim 环境 |
| Pegasus 配置 | `~/PegasusSimulator/.../config/configs.yaml` | Pegasus 参数 |

---

## 🚀 我创建的脚本详解

### 1. sensor_test.py ⭐推荐

**路径**: `~/PegasusSimulator/examples/sensor_test.py`

**功能**:
- 前置 RGB+深度相机 (640x480, 30Hz)
- LiDAR (Example_Rotary, 10Hz)
- 3 个彩色障碍物 (红、蓝、绿方块)
- 不依赖 ROS2，避免 Python 版本冲突

**运行**:
```bash
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/sensor_test.py
```

### 2. sensor_ros2.py

**路径**: `~/PegasusSimulator/examples/sensor_ros2.py`

**功能**:
- 与 sensor_test.py 相同的传感器配置
- 尝试通过 ROS2 发布传感器数据
- ⚠️ 有 Python 版本冲突问题 (Isaac Sim Python 3.11 vs ROS2 Python 3.12)

### 3. start_sensor_sim.sh

**路径**: `~/start_sensor_sim.sh`

**功能**:
- 设置 Isaac Sim 内置 ROS2 环境变量
- 启动 sensor_test.py

**内容**:
```bash
#!/bin/bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib
cd ~/PegasusSimulator
$ISAACSIM_PYTHON examples/sensor_test.py
```

---

## 🔧 环境变量配置 (~/.bashrc)

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

---

## 🗑️ 可删除的文件 (节省空间)

| 文件 | 路径 | 大小 | 说明 |
|------|------|------|------|
| Isaac Sim 安装包 | `~/isaacsim/isaac-sim-standalone-5.1.0-linux-x86_64.zip` | ~8 GB | 已解压，可删除 |
| Isaac Sim 内部 zip | `~/isaacsim/isaac-sim-5.1.0.zip` | ~8 GB | 可删除 |
| CUDA 密钥包 | `~/cuda-keyring_1.1-1_all.deb` | ~10 KB | 已安装，可删除 |

```bash
# 清理命令 (可选)
rm ~/isaacsim/isaac-sim-standalone-5.1.0-linux-x86_64.zip
rm ~/isaacsim/isaac-sim-5.1.0.zip
rm ~/cuda-keyring_1.1-1_all.deb
```

---

**文档创建日期**: 2026-01-18
