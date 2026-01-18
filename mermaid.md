graph TD
    subgraph Host_Computer ["🖥️ 主机环境 (Ubuntu 24.04 LTS)"]
        style Host_Computer fill:#f0f0f0,stroke:#333,stroke-width:2px,color:#000

        subgraph Infrastructure ["⚙️ 基础设施层"]
            style Infrastructure fill:#e4e4e4,stroke:#666,color:#000
            NVIDIA["NVIDIA Driver 580.x\nCUDA 13.0\n(GPU 加速核心)"]
            GCC["GCC/G++ 11.5.0\n(编译器)"]
        end

        subgraph Simulation_World ["🌏 虚拟世界层 - Isaac Sim 5.1.0"]
            style Simulation_World fill:#d4edda,stroke:#28a745,stroke-width:2px,color:#000
            
            Isaac_Core["Isaac Sim 核心\n(PhysX 5 物理引擎 / RTX 渲染)"]
            
            subgraph Isaac_Sensors ["📷 Isaac Sim 传感器仿真"]
                style Isaac_Sensors fill:#c3e6cb,stroke:#28a745,color:#000
                Camera_Sim["RGB/深度相机\n(RTX 渲染)"]
                LiDAR_Sim["LiDAR 点云\n(Ouster/Velodyne/...)"]
                Radar_Sim["毫米波雷达\n(可选)"]
            end

            Isaac_Core --> Camera_Sim
            Isaac_Core --> LiDAR_Sim
            Isaac_Core --> Radar_Sim
        end

        subgraph Pegasus_Layer ["🌉 Pegasus Simulator 5.1.0 (仿真桥梁)"]
            style Pegasus_Layer fill:#fff3cd,stroke:#ffc107,stroke-width:2px,color:#000
            
            Pegasus_Core["Pegasus 核心\n(无人机模型/物理状态获取)"]
            
            subgraph Nav_Sensor_Attack ["🎯 导航传感器欺骗 (攻击 EKF2)"]
                style Nav_Sensor_Attack fill:#f8d7da,stroke:#dc3545,stroke-width:2px,color:#000
                Nav_Sensors["导航传感器模型\n(IMU/GPS/气压计/磁力计)"]
                Nav_Spoof["🔴 导航欺骗注入点\npx4_mavlink_backend.py"]
            end
            
            subgraph Perception_Attack ["🎯 感知传感器欺骗 (研究重点!)"]
                style Perception_Attack fill:#f5c6cb,stroke:#dc3545,stroke-width:3px,color:#000
                Perception_Sensors["感知传感器模型\n(monocular_camera.py / lidar.py)"]
                Perception_Spoof["🔴 感知欺骗注入点\n(图像注入/点云注入)"]
            end

            Pegasus_Core --> Nav_Sensors
            Pegasus_Core --> Perception_Sensors
            Nav_Sensors --> Nav_Spoof
            Perception_Sensors --> Perception_Spoof
            Nav_Spoof --> MAVLink_Out(["MAVLink 发送\nHIL_SENSOR/HIL_GPS"])
        end

        subgraph Flight_Control ["🧠 飞行控制层 - PX4 v1.15.4 SITL"]
            style Flight_Control fill:#ffeeba,stroke:#856404,stroke-width:2px,color:#000
            
            PX4_SITL["PX4 SITL\n(软件在环仿真)"]
            
            subgraph PX4_Core ["PX4 内部"]
                style PX4_Core fill:#fff,stroke:#856404,color:#000
                EKF2["😵 EKF2 状态估计器\n(导航欺骗受害者)"]
                Controllers["控制器级联\n(位置→姿态→角速度)"]
                Mixer["混控器\n(电机 PWM)"]
            end
            
            PX4_SITL --> EKF2
            EKF2 --> Controllers
            Controllers --> Mixer
        end

        subgraph Perception_System ["👁️ 感知系统层 (感知欺骗目标)"]
            style Perception_System fill:#f8d7da,stroke:#dc3545,stroke-width:2px,color:#000
            Obstacle_Detection["😵 避障系统\n(感知欺骗受害者)"]
            Object_Detection["😵 目标检测\n(感知欺骗受害者)"]
            SLAM["😵 SLAM\n(感知欺骗受害者)"]
        end

        subgraph Middleware ["📡 通信中间件 - ROS2 JAZZY"]
            style Middleware fill:#cce5ff,stroke:#007bff,stroke-width:2px,color:#000
            ROS2["ROS2 话题总线\n⚠️ Python 3.12 与 Isaac Sim 3.11 冲突"]
        end

        subgraph Research ["📊 研究分析层"]
            style Research fill:#e2e3e5,stroke:#383d41,color:#000
            Analysis["分析脚本\n(数据记录/攻击效果评估)"]
        end

        %% ========== 数据流连接 ==========

        %% 物理仿真
        Isaac_Core -- "物理计算" --> NVIDIA
        Pegasus_Core -- "获取物理状态" --> Isaac_Core
        Camera_Sim --> Perception_Sensors
        LiDAR_Sim --> Perception_Sensors

        %% MAVLink 闭环 (导航)
        MAVLink_Out == "HIL_SENSOR/GPS\n(TCP 4560)" ==> PX4_SITL
        Mixer -- "HIL_ACTUATOR\n(电机指令)" --> Pegasus_Core
        Pegasus_Core -- "应用推力" --> Isaac_Core

        %% 感知数据流
        Perception_Spoof -.-> |"欺骗后的\n图像/点云"| Obstacle_Detection
        Perception_Spoof -.-> |"欺骗后的\n图像/点云"| Object_Detection
        Perception_Spoof -.-> |"欺骗后的\n图像/点云"| SLAM

        %% ROS2 数据流
        PX4_SITL -.-> |"/fmu/out/..."| ROS2
        Perception_Sensors -.-> |"传感器数据"| ROS2
        ROS2 -.-> Analysis

    end

    %% 图例
    classDef attack fill:#f8d7da,stroke:#dc3545,stroke-width:2px
    classDef victim fill:#ffeeba,stroke:#dc3545,stroke-width:2px
    