# PGFUZZ: Policy-Guided Fuzzing for Robotic Vehicles

## 完整学习总结

---

# 第一部分：论文基本信息

## 1.1 论文元数据

| 项目 | 内容 |
|------|------|
| 标题 | PGFUZZ: Policy-Guided Fuzzing for Robotic Vehicles |
| 会议 | Network and Distributed Systems Security (NDSS) Symposium 2021 |
| 作者 | Hyungsub Kim, Muslum Ozgur Ozmen, Antonio Bianchi, Z. Berkay Celik, Dongyan Xu |
| 机构 | Purdue University |
| 代码 | https://github.com/purseclab/PGFUZZ |

## 1.2 论文核心贡献（原文 Section I）

论文明确列出三个贡献：

1. **Behavior-aware Bug Oracle**：通过时序逻辑公式（MTL）识别和形式化表达 RV 的安全与功能需求策略，利用这些策略发现导致策略违反的 bug。

2. **Policy-Guided Mutation Engine**：设计新型 fuzzing 方法，通过 (i) 变异输入/参数来否定安全策略，使用专门的距离度量作为启发式，以及 (ii) 最小化与被分析策略相关的输入和参数的 fuzzing 空间。

3. **Evaluation in real-world RVs**：在三个最流行的飞行控制软件（ArduPilot、PX4、Paparazzi）上应用 PGFUZZ，发现 156 个此前未知的 bug，其中 106 个已被开发者确认。

---

# 第二部分：研究背景与问题定义

## 2.1 RV 控制系统背景（原文 Section II）

### 2.1.1 RV 的输入输出

RV（Robotic Vehicle，机器人车辆）的控制软件主要操作三种类型的输入：

| 输入类型 | 符号 | 说明 | 示例 |
|----------|------|------|------|
| 配置参数 | InputP | 配置 RV 运行方式的数值参数 | PID 控制器的 KP、Ki、Kd 参数 |
| 用户命令 | InputC | 用户动态操作 RV 的命令 | Disarming（停止电机）命令 |
| 环境因素 | InputE | 影响系统输出的外部因素 | 风速、传感器噪声 |

### 2.1.2 控制算法工作流程（原文 Figure 1）

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         RV 控制软件工作流程                              │
│                                                                         │
│   InputC ──┐      ┌──────────────────────────────────────┐              │
│  用户命令   │      │           Cyber Space                │              │
│            │      │  ┌─────────────────────────────────┐ │              │
│   InputP ──┼──────│─▶│      Control Algorithm          │ │              │
│  配置参数   │      │  │                                 │ │              │
│            │      │  │  Reference states r(t)          │ │              │
│   InputE ──┘      │  │         │                       │ │              │
│  环境因素          │  │         ▼                       │ │              │
│                   │  │  ┌───────────────┐              │ │              │
│                   │  │  │   Σ (误差计算)  │ e(t)=r(t)-y(t)│              │
│                   │  │  └───────┬───────┘              │ │              │
│                   │  │          ▼                      │ │              │
│                   │  │  ┌───────────────┐              │ │              │
│                   │  │  │  P  │  I  │ D │ (PID控制)    │ │              │
│                   │  │  └───────┬───────┘              │ │              │
│                   │  │          │ System inputs u(t)   │ │              │
│                   │  └──────────┼──────────────────────┘ │              │
│                   └─────────────┼────────────────────────┘              │
│                                 │                                       │
│                                 ▼                                       │
│                   ┌─────────────────────────────────────┐               │
│                   │         Physical Space              │               │
│                   │  ┌───────────────────────────────┐  │               │
│                   │  │  Commands to actuators        │  │               │
│                   │  │         │                     │  │               │
│                   │  │         ▼                     │  │               │
│                   │  │  ┌─────────────┐              │  │               │
│                   │  │  │   Motors    │              │  │               │
│                   │  │  └──────┬──────┘              │  │               │
│                   │  │         │                     │  │               │
│                   │  │         ▼                     │  │               │
│                   │  │  Measured system outputs y(t) │  │               │
│                   │  │         │                     │  │               │
│                   │  │  ┌──────┴──────┐              │  │               │
│                   │  │  │   Sensors   │──────────────┼──┼───▶ Feedback  │
│                   │  │  └─────────────┘              │  │      to Cyber │
│                   │  └───────────────────────────────┘  │               │
│                   └─────────────────────────────────────┘               │
│                                                                         │
│   坐标系说明：                                                           │
│   - X 轴：Roll（横滚）                                                   │
│   - Y 轴：Pitch（俯仰）                                                  │
│   - Z 轴：Yaw（偏航）                                                    │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

控制算法周期性执行三个步骤：
1. 控制算法读取传感器测量的系统输出 y(t)
2. 算法计算误差 e(t) = r(t) - y(t)，其中 r(t) 是参考状态，t 是当前时间
3. PID 控制算法通过 e(t) 导出系统输入 u(t)

### 2.1.3 Fuzzing 的两个核心问题（原文 Section II）

论文指出传统 fuzzing 在 RV 测试中面临的两个核心问题：

| 问题 | 传统 Fuzzing 的处理方式 | RV Fuzzing 的挑战 |
|------|------------------------|------------------|
| **输入生成** | 完全随机或代码覆盖引导 | RV 输入空间巨大（数百参数），需要智能引导 |
| **Bug Oracle** | 程序崩溃（内存错误） | RV 的 bug 通常不导致程序崩溃，而是物理状态异常 |

## 2.2 Motivating Example：降落伞策略（原文 Section III）

### 2.2.1 策略描述

ArduPilot 官方文档规定，释放降落伞必须满足以下四个条件：

```
条件 1: 电机必须已启动（motors must be armed）
条件 2: 飞行器不能处于 FLIP 或 ACRO 飞行模式
条件 3: 气压计必须显示飞行器没有在爬升
条件 4: 飞行器当前高度必须高于 CHUTE_ALT_MIN 参数值
```

### 2.2.2 MTL 形式化表达

论文将上述策略表达为 MTL（Metric Temporal Logic）公式：

```
□ {(Parachute=on)} → {(Armed=true) ∧ 
                      (Mode_t ≠ FLIP/ACRO) ∧ 
                      (ALT_t ≤ ALT_{t-1}) ∧ 
                      (ALT_t > CHUTE_ALT_MIN)}
```

其中：
- `□` 表示 "Always"（在所有时间点）
- `t` 和 `t-1` 表示当前时间和上一时刻
- `ALT` 表示高度（Altitude）

### 2.2.3 传统方法的局限性

论文指出传统 fuzzing 方法无法发现此类 bug 的两个原因：

**原因 1**：传统方法不考虑 RV 控制软件的完整输入空间

> "First, policy violations are often triggered by the composition of different types of system inputs. However, these approaches only focus on a single part of the input space, meaning they do not consider unified behavior of user commands, configuration parameters, and environmental factors."

**原因 2**：传统方法的 bug oracle 只检测特定类型的 bug

> "Second, their bug oracles are designed to detect specific bug types, such as deviated flight paths or instability. To detail, if a policy violation causes unexpected physical behavior, e.g., failing to trigger a GPS fail-safe mode, their bug oracles cannot detect such undesired behavior."

### 2.2.4 PGFUZZ 发现的实际 bug

> "By using PGFUZZ, we found that ArduPilot improperly checks the first three requirements. This leads to a policy violation where the vehicle deploys the parachute when it is climbing, causing it to crash on the ground."

ArduPilot 只检查了第 4 个条件（高度），而忽略了前 3 个条件。

## 2.3 威胁模型（原文 Section III - Threat Model）

### 2.3.1 攻击者能力

论文考虑两种来源的威胁：

**设计缺陷（来自良性开发者和用户）**：
- 参数文档不完善
- 未预期的环境条件
- 代码 bug

**恶意意图（来自攻击者）**：
攻击者可以控制 RV 的三种输入：

| 输入类型 | 攻击方式 |
|----------|----------|
| 配置参数 (InputP) | 在飞行前覆盖或飞行后更改（类似 RVFuzzer） |
| 用户命令 (InputC) | 利用 RV 通信协议漏洞重放或伪造命令 |
| 环境因素 (InputE) | 操纵环境条件或等待合适条件发起攻击 |

### 2.3.2 攻击者目标

> "The adversary's goal is to physically impact the RV's operations (e.g., causing a physical crash or disrupting the RV's camera) by stealthily triggering policy violations."

攻击者目标是通过隐蔽地触发策略违反来影响 RV 的物理操作。

### 2.3.3 不在范围内的攻击

论文明确排除以下攻击类型：

1. **自毁性输入**（如直接停止执行器）：
   > "We note that an adversary could also simply drop or disarm the vehicle by sending a malicious command (e.g., stopping actuators); however, these attacks are not stealthy."

2. **物理传感器攻击**（如 GPS 和陀螺仪欺骗）：
   > "The main reasons are (1) the root causes of sensor attacks arise in the hardware components (e.g., acoustic attacks against gyroscope), rather than buggy code in the vehicle's control program."

3. **恶意代码注入**：
   > "There exist effective techniques to detect sensor and code injection attacks."

---

# 第三部分：PGFUZZ 方法设计

## 3.1 设计挑战（原文 Section IV-A）

论文指出传统 fuzzing 技术在 RV 策略引导 fuzzing 中的两个主要局限：

### 局限 1：Bug Oracle 不适用

> "First, their bug oracles are not designed to detect undesired system states that do not cause a system crash, memory-access violation, or physical instability."

传统 bug oracle 无法检测不导致崩溃、内存访问违规或物理不稳定的异常系统状态。

**解决方案**：Behavior-aware Bug Oracle
- 通过 MTL 公式感知 RV 的期望状态
- 在 fuzzing 过程中检测公式是否被违反

### 局限 2：变异引擎不智能

> "This limitation is due to the large input space of the RVs, with tens of different parameters and commands, each of which can have a wide range of values."

RV 输入空间巨大，传统变异引擎无法智能生成输入。

**解决方案**：Policy-Guided Mutation Engine
- 建立策略中每个术语与影响 RV 状态的输入之间的映射
- 使用距离度量衡量当前状态与策略违反之间的"距离"

## 3.2 系统架构（原文 Section IV-B 和 Figure 2）

PGFUZZ 包含三个相互连接的组件：

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              PGFUZZ 系统架构                                     │
│                                                                                 │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                        1) Pre-Processing 预处理                            │  │
│  │                                                                           │  │
│  │  ┌─────────┐     ┌───────────────┐     ┌─────────────────────────────┐   │  │
│  │  │Documents│────▶│ Extract MTL   │────▶│ Profiling Engine            │   │  │
│  │  │ 文档    │     │ Policies      │     │                             │   │  │
│  │  └─────────┘     │ 提取MTL策略    │     │ ┌─────────────────────────┐ │   │  │
│  │                  └───────────────┘     │ │ Static Analysis (DFG)   │ │   │  │
│  │  ┌─────────┐           │              │ │ 静态分析                 │ │   │  │
│  │  │Source   │───────────┘              │ └─────────────────────────┘ │   │  │
│  │  │Code     │                          │ ┌─────────────────────────┐ │   │  │
│  │  │(可选)   │                          │ │ Dynamic Analysis        │ │   │  │
│  │  └─────────┘                          │ │ 动态分析                 │ │   │  │
│  │                                       │ └─────────────────────────┘ │   │  │
│  │  ┌─────────┐                          └──────────────┬──────────────┘   │  │
│  │  │Input_set│                                         │                  │  │
│  │  │ InputC  │─────────────────────────────────────────┤                  │  │
│  │  │ InputP  │                                         ▼                  │  │
│  │  │ InputE  │                              ┌─────────────────────┐       │  │
│  │  └─────────┘                              │ Input-Policy Maps   │       │  │
│  │                                           │ (command2, policy1) │       │  │
│  │                                           │ (param7, policy2)   │       │  │
│  │                                           │ (environ3, policy2) │       │  │
│  │                                           └──────────┬──────────┘       │  │
│  └──────────────────────────────────────────────────────┼──────────────────┘  │
│                                                         │                     │
│                                                         ▼                     │
│  ┌───────────────────────────────────────────────────────────────────────────┐│
│  │                    2) Policy-Guided Fuzzing 策略引导模糊测试               ││
│  │                                                                           ││
│  │  ┌────────────────┐      ┌──────────────────────────────────────────┐    ││
│  │  │Mutation Engine │      │              Simulator                   │    ││
│  │  │   变异引擎      │      │                模拟器                    │    ││
│  │  │                │      │                                          │    ││
│  │  │ Input-Policy   │      │  ┌──────────┐                            │    ││
│  │  │ Maps           │ Send │  │          │  States measured           │    ││
│  │  │                │ Input│  │  Vehicle │  from sensors              │    ││
│  │  │                │─────▶│  │          │────────┐                   │    ││
│  │  │                │      │  └──────────┘        │                   │    ││
│  │  └───────▲────────┘      └──────────────────────┼───────────────────┘    ││
│  │          │                                      │                        ││
│  │          │ Send each                            ▼                        ││
│  │          │ propositional            ┌───────────────────────┐            ││
│  │          │ distance                 │ Noise Elimination     │            ││
│  │          │                          │ 噪声消除               │            ││
│  │          │                          └───────────┬───────────┘            ││
│  │          │                                      │ Filtered states        ││
│  │          │                                      ▼                        ││
│  │          │                          ┌───────────────────────┐            ││
│  │          │                          │ Calculate Distance    │            ││
│  │          │                          │ Metrics 计算距离度量   │            ││
│  │          │                          └───────────┬───────────┘            ││
│  │          │                                      │                        ││
│  │          │                                      ▼                        ││
│  │          │                          ┌───────────────────────┐            ││
│  │          └──────────────────────────│ Check Policy Violation│            ││
│  │                                     │ 检查策略违反           │            ││
│  │                                     │                       │            ││
│  │                                     │ Policy and MTL        │            ││
│  │                                     │ Formulas              │            ││
│  │                                     └───────────┬───────────┘            ││
│  │                                                 │                        ││
│  │                                                 │ If violated            ││
│  │                                                 ▼                        ││
│  │                                     ┌───────────────────────┐            ││
│  │                                     │ A set of inputs which │            ││
│  │                                     │ cause policy violation│            ││
│  │                                     └───────────┬───────────┘            ││
│  └─────────────────────────────────────────────────┼────────────────────────┘│
│                                                    │                         │
│                                                    ▼                         │
│  ┌───────────────────────────────────────────────────────────────────────────┐│
│  │                    3) Bug Post-Processing Bug后处理                       ││
│  │                                                                           ││
│  │  ┌─────────────────────┐          ┌─────────────────────────┐            ││
│  │  │      Bug Pool       │─────────▶│   Input Minimization    │            ││
│  │  │      Bug 池         │          │     输入最小化           │            ││
│  │  └─────────────────────┘          └─────────────────────────┘            ││
│  │                                                                           ││
│  └───────────────────────────────────────────────────────────────────────────┘│
│                                                                                │
└────────────────────────────────────────────────────────────────────────────────┘
```

## 3.3 预处理阶段（原文 Section V-A）

### 3.3.1 MTL 策略提取

#### MTL 语法定义

MTL（Metric Temporal Logic）公式由以下元素组成：

| 元素 | 符号 | 说明 |
|------|------|------|
| 原子命题 | p ∈ AP | 由"术语"组成的逻辑语句，术语可以是 RV 物理状态、配置参数或环境因素 |
| 命题逻辑算子 | ∧, ∨, ¬ | 与、或、非 |
| 时序算子 | □_I, ◇_I, U_I, ○_I | Always、Eventually、Until、Next，I 表示时间区间 |

形式化定义：
```
φ ::= ⊤ | p | ¬φ | φ1 ∨ φ2 | φ1 U_I φ2 | ○_I φ
```

其中 p ∈ AP，⊤ = true。

#### 策略模板（原文 Table I）

论文提供三种策略模板：

| ID | 模板描述 | MTL 表示 |
|----|----------|----------|
| T1 | term_j 应在 term_i 满足后 k 时间内为真 | □(term_i → ◇_{[0,k]} term_j) |
| T2 | 如果 term_i 为真，则 term_j...term_n 也为真，且 term_k...term_m 为假 | □(term_i → (term_j ∧...∧ term_n) ∧ ¬(term_k ∧...∧ term_m)) |
| T3 | 如果 term_i...term_n 为真，则 term_j 也为真 | □((term_i ∧...∧ term_n) → term_j) |

#### 策略提取过程

> "We manually identify the policies through requirements defined in documentation and comments in the source code of popular RVs, ArduPilot, PX4, and Paparazzi. The policies are extracted in natural language and then expressed with MTL formulas."

策略提取时间统计（原文 Section V-A1）：
- ArduPilot：7.5 小时（两位作者），30 条策略
- PX4：3.5 小时，21 条策略
- Paparazzi：2.4 小时，5 条策略

### 3.3.2 Profiling Engine（分析引擎）

#### 目标

减少巨大的输入空间。论文给出 ArduPilot v.4.0.3 的输入规模：
- 配置参数 (InputP)：1,140 个
- 用户命令 (InputC)：58 个
- 环境因素 (InputE)：168 个

#### 六步分析流程（原文 Figure 3）

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        Profiling Engine 六步流程                                 │
│                                                                                 │
│  步骤 ①: Policy-Term Mapping（策略-术语映射）                                    │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │ 将策略分解为术语列表                                                       │   │
│  │                                                                          │   │
│  │ 示例：A.CHUTE1 策略                                                       │   │
│  │ ┌────────────────────────────────────────────────────┐                   │   │
│  │ │ Policy-term map                                    │                   │   │
│  │ │ policy      | related terms                        │                   │   │
│  │ │ A.CHUTE1    | Chute, Mode, Alt, Armed, ...        │                   │   │
│  │ └────────────────────────────────────────────────────┘                   │   │
│  │                                                                          │   │
│  │ 术语类型：                                                                │   │
│  │ - 物理状态：parachute, armed, mode, altitude                             │   │
│  │ - 配置参数：CHUTE_ALT_MIN                                                 │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                           │                                     │
│                                           ▼                                     │
│  步骤 ②: Static Analysis（静态分析）                                            │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │ 通过静态分析识别与每个配置参数相关的术语                                     │   │
│  │                                                                          │   │
│  │ 方法：                                                                    │   │
│  │ 1. 将配置参数映射到源代码中的变量                                          │   │
│  │ 2. 构建 def-use 链来映射参数到 MTL 公式中的术语                            │   │
│  │ 3. 使用同义词表匹配源代码变量名和 MTL 术语名                                │   │
│  │                                                                          │   │
│  │ 示例（原文 Figure 5）：                                                    │   │
│  │ ┌────────────────────────────────────────────────────────────────┐       │   │
│  │ │ 源代码：                                                        │       │   │
│  │ │ 1  AP_GROUPINFO("TEMP", ..., ground_temp);                     │       │   │
│  │ │ 2  _user_temp = ground_temp + 273.15f;                         │       │   │
│  │ │ ...                                                            │       │   │
│  │ │ 315 temp = _user_temp;                                         │       │   │
│  │ │ 320 altitude = 153.8462f * temp * ...                          │       │   │
│  │ │                                                                │       │   │
│  │ │ Def-use chain:                                                 │       │   │
│  │ │ (1) [ground_temp, line 1, line 2]                              │       │   │
│  │ │ (2) [_user_temp, line 2, line 315]                             │       │   │
│  │ │ (3) [temp, line 315, line 320]                                 │       │   │
│  │ │                                                                │       │   │
│  │ │ 结论：TEMP 参数与 altitude 状态相关！                            │       │   │
│  │ └────────────────────────────────────────────────────────────────┘       │   │
│  │                                                                          │   │
│  │ 输出：Parameter-term map                                                  │   │
│  │ ┌────────────────────────────────────────────────────┐                   │   │
│  │ │ parameter   | related terms                        │                   │   │
│  │ │ ABS_PRESS   | Alt, Baro, ...                       │                   │   │
│  │ └────────────────────────────────────────────────────┘                   │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                           │                                     │
│                                           ▼                                     │
│  步骤 ③: Dependency Analysis（依赖分析）                                        │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │ 分析 InputP、InputC、InputE 之间的依赖关系                                 │   │
│  │                                                                          │   │
│  │ 示例：                                                                    │   │
│  │ Parachute 命令依赖于 CHUTE_ENABLED 参数                                   │   │
│  │ （只有 CHUTE_ENABLED=true 时，Parachute 命令才能执行）                     │   │
│  │                                                                          │   │
│  │ 输出：Dependency among inputs                                             │   │
│  │ ┌────────────────────────────────────────────────────┐                   │   │
│  │ │ InputP,E,C  | dependency                           │                   │   │
│  │ │ Parachute   | CHUTE_ENABLED, ...                   │                   │   │
│  │ └────────────────────────────────────────────────────┘                   │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                           │                                     │
│                                           ▼                                     │
│  步骤 ④: Dynamic Analysis（动态分析）                                           │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │ 使用 RV 模拟器进行动态分析，完成两个任务：                                   │   │
│  │                                                                          │   │
│  │ 任务 1：过滤只读和不支持的 InputP                                          │   │
│  │ 任务 2：映射 InputC 和 InputE 到策略术语                                   │   │
│  │                                                                          │   │
│  │ 方法：                                                                    │   │
│  │ 1. 记录无输入时每种飞行模式下所有状态值一分钟                               │   │
│  │ 2. 计算每个状态的标准差 SD{State(i)}                                      │   │
│  │ 3. 执行某个 input_j，记录状态变化，计算 SD{State(i,j)}                    │   │
│  │ 4. 如果 |SD{State(i)} - SD{State(i,j)}| > SD{State(i)}                  │   │
│  │    则该 input_j 影响该状态                                                │   │
│  │                                                                          │   │
│  │ 输出（原文 Figure 6 - throttle 命令的分析结果）：                           │   │
│  │ throttle 命令影响的状态：Heading, Throttle, Altitude, Climb               │   │
│  │                                                                          │   │
│  │ 输出 Maps：                                                               │   │
│  │ - InputP-term map                                                        │   │
│  │ - InputE-term map                                                        │   │
│  │ - InputC-term map                                                        │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                           │                                     │
│                                           ▼                                     │
│  步骤 ⑤: Extracting Inputs Related to Each Policy（提取每个策略的相关输入）     │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │ 从步骤④的三个 map 中提取每个策略的相关输入                                  │   │
│  │                                                                          │   │
│  │ 输出：Input-policy map                                                    │   │
│  │ ┌────────────────────────────────────────────────────┐                   │   │
│  │ │ policy      | related inputs                       │                   │   │
│  │ │ A.CHUTE1    | Wind_speed, Parachute, Throttle, ...│                   │   │
│  │ └────────────────────────────────────────────────────┘                   │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                           │                                     │
│                                           ▼                                     │
│  步骤 ⑥: Analyzing Time Constraints（分析时间约束）                             │
│  ┌──────────────────────────────────────────────────────────────────────────┐   │
│  │ 确定 MTL 公式中未知的时间限制 k                                            │   │
│  │                                                                          │   │
│  │ 示例：A.BRAKE1 策略                                                       │   │
│  │ □{(Mode_t = BRAKE) → (◇_{[0,k]} Pos_t = Pos_{t-1})}                      │   │
│  │ "当飞行器处于 BRAKE 模式时，必须在 k 秒内停止"                              │   │
│  │                                                                          │   │
│  │ 方法：                                                                    │   │
│  │ 1. 将策略分解为术语                                                       │   │
│  │ 2. 随机选择策略相关输入并执行                                             │   │
│  │ 3. 使飞行器满足前置条件，测量满足期望状态所需时间 k                         │   │
│  │ 4. 重复 100 次，k 定义为最大所需时间                                       │   │
│  │                                                                          │   │
│  │ 示例结果：BRAKE 模式需要最多 12.7 秒保持在同一位置                          │   │
│  └──────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## 3.4 策略引导 Fuzzing（原文 Section V-B）

### 3.4.1 算法概述（原文 Algorithm 1）

```
Algorithm 1: Policy-Guided Fuzzing
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: 模拟器 SIM, 最小化输入空间 Input_min, 输入-策略映射 MAP, 
       MTL 公式 φ, fuzzing 时间限制 τ
Output: 策略违反 V 和导致违反的输入序列 V_set
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1:  function FUZZING(SIM, MAP, Input_min, φ, τ)          ▷ Main
2:      input_seq = ∅                                    ▷ 初始化输入序列
3:      while V = ∅ or total_time < τ do
4:          input ← MUTATE(MAP, Input_min, φ, DIS)       ▷ 获取变异输入
5:          S ← SIM.execute(input)                       ▷ 从 SIM 收集 RV 状态
6:          S ← NOISE.elimination(S)                     ▷ 消除环境噪声
7:          DIS ← UPDATE_DISTANCE(φ, S)                  ▷ 计算距离
8:          V ← POLICY_CHECK(φ, DIS)                     ▷ 检查策略违反
9:          input_seq = input_seq ∪ input
10:     end while
11:     V_set ← POST_BUG(input_seq, V)                   ▷ 执行 Bug 后处理
12:     return ⟨V, V_set⟩
13: end function

14: function MUTATE(MAP, Input_min, φ, DIS)              ▷ 通过 MTL 变异输入
15:     input ← RANDOM(Input_min, MAP)                   ▷ 随机选择一个输入
16:     input ← GUIDANCE(input, φ, DIS)                  ▷ 根据 DIS 选择值
17:     return input
18: end function
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 3.4.2 噪声消除（原文 Section V-B2）

#### 问题

> "For instance, let us assume our mutation engine's goal is to minimize the altitude (to trigger a policy violation) and at time T=1, Alt=15. Then, PGFUZZ executes an input to decelerate the motors' speed. This input decreases the altitude under normal conditions. However, if strong wind and sensor noise occur together with the input, the altitude increases by 2 meters, T=2, Alt=17. Then, PGFUZZ wrongly determines that executing the input to decelerate the motors' speed increases the altitude."

环境噪声（如风和传感器噪声）可能导致 PGFUZZ 错误判断输入对状态的影响。

#### 解决方案

> "To address this, we use reference state values in the control algorithms of flight control software."

使用控制算法中的参考状态值和移动平均来消除噪声：
```
Moving_average(State_i_act + State_i_err)
```

其中 `State_i_err = State_i_ref - State_i_act`

### 3.4.3 策略检查器与距离度量（原文 Section V-B3）

#### 距离度量的两种类型

| 类型 | 作用 |
|------|------|
| 命题距离（Propositional Distance） | 引导变异引擎 |
| 全局距离（Global Distance） | 通知 bug oracle 策略是否被违反 |

#### 公式转换

策略检查器首先将 MTL 公式从 "always" 形式转换为 "not eventually" 形式。

**降落伞策略的转换过程**：

原始公式：
```
□ {(Parachute=on)} → {(Armed=true) ∧ (Mode_t ≠ FLIP/ACRO) ∧ 
                      (ALT_t ≤ ALT_{t-1}) ∧ (ALT_t > CHUTE_ALT_MIN)}
```

转换后：
```
¬◇ [{(Parachute=on)} ∧ {(Armed ≠ true) ∨ (Mode_t = FLIP/ACRO) ∨ 
                        (ALT_t > ALT_{t-1}) ∨ (ALT_t < CHUTE_ALT_MIN)}]
```

#### 命题距离计算规则

**布尔值命题**：
```
距离 = +1  如果命题为真
距离 = -1  如果命题为假
```

**数值命题**：
```
距离 = 归一化差值

例：(ALT_t > CHUTE_ALT_MIN) 的距离
    = (CHUTE_ALT_MIN - ALT_t) / CHUTE_ALT_MIN
```

**降落伞策略的五个命题距离**（原文公式）：

```
         ┌ +1  如果 Chute_t = on
(1) P1 = ┤
         └ -1  如果 Chute_t ≠ on

         ┌ +1  如果 Armed_t ≠ true
(2) P2 = ┤
         └ -1  如果 Armed_t = true

         ┌ +1  如果 Mode_t = FLIP/ACRO
(3) P3 = ┤
         └ -1  如果 Mode_t ≠ FLIP/ACRO

(4) P4 = (ALT_t - ALT_{t-1}) / ALT_t

(5) P5 = (CHUTE_ALT_MIN - ALT_t) / CHUTE_ALT_MIN
```

#### 全局距离计算规则

逻辑算子到算术运算的转换：

| 逻辑算子 | 算术运算 |
|----------|----------|
| NOT (¬) | × (-1) |
| AND (∧) | min() |
| OR (∨) | max() |

**降落伞策略的全局距离公式**：
```
Global Distance = -1 × [min{P1, max(P2, P3, P4, P5)}]
```

**判定规则**：
- 全局距离 > 0：策略满足
- 全局距离 < 0：策略违反（发现 bug）

### 3.4.4 变异引擎（原文 Section V-B4）

#### 核心策略

> "The mutation engine feeds inputs to the simulator to minimize the global distance, where the negative values of the global distance indicate a policy violation."

变异引擎的目标是最小化全局距离，使其变为负值。

#### 变异过程

论文描述的四个步骤：

> "(1) It first randomly selects an input from the Input_min of the target policy, which is stored in the input-policy map. Then, it randomly selects a value and assigns it to the input."

> "(2) It executes the selected input on the simulator, computes the propositional and global distances, and flags a policy violation if the global distance becomes negative."

> "(3) If the executed input increases the propositional distance, the mutation engine stores the input with the assigned value."

> "(4) When the mutation engine randomly selects the stored input again (e.g., changing altitude), it applies the stored value to the input instead of randomly assigning a new value to the input."

```
变异引擎四步过程：

步骤 1: 从 Input_min 随机选择一个输入，随机赋值

步骤 2: 在模拟器中执行，计算命题距离和全局距离
        如果全局距离 < 0，标记策略违反

步骤 3: 如果执行的输入增加了命题距离，存储 (input, value) 配对
        例：如果增加高度导致 P4 增加，存储 (altitude, increase)

步骤 4: 当再次选择已存储的输入时，使用存储的值而非随机值
```

### 3.4.5 完整工作示例（原文 Table II）

测试降落伞策略 A.CHUTE1（CHUTE_ALT_MIN = 100 米）：

| 时间 | Parachute | Armed | FLIP/ACRO | 测量高度 | 过滤高度 | P1 | P2 | P3 | P4 | P5 | 全局距离 | 下一步 Fuzzed 输入 |
|------|-----------|-------|-----------|----------|----------|-----|-----|-----|------|------|----------|-------------------|
| T=1 | off | on | false | 92m | 94m | -1 | -1 | -1 | 0 | 0.06 | +1 | WIND_SPEED = 5 |
| T=2 | off | on | false | 93m | 95m | -1 | -1 | -1 | 0.01 | 0.05 | +1 | Parachute = on |
| T=3 | off | on | false | 95m | 95m | -1 | -1 | -1 | 0 | 0.05 | +1 | Increase throttle |
| T=4 | off | on | false | 97m | 99m | -1 | -1 | -1 | 0.04 | 0.01 | +1 | WIND_SPEED = 5 |
| T=5 | off | on | false | 102m | 104m | -1 | -1 | -1 | 0.05 | -0.04 | +1 | Parachute = on |
| T=6 | on | on | false | 106m | 106m | +1 | -1 | -1 | 0.02 | -0.06 | **-0.02** | - |

**分析 T=6 时的计算**：
```
P1 = +1  (降落伞已释放)
P2 = -1  (Armed = true，不满足违反条件)
P3 = -1  (Mode ≠ FLIP/ACRO，不满足违反条件)
P4 = +0.02 (正在爬升！ALT_t > ALT_{t-1})
P5 = -0.06 (高度足够 > CHUTE_ALT_MIN)

max(P2, P3, P4, P5) = max(-1, -1, +0.02, -0.06) = +0.02
min(P1, +0.02) = min(+1, +0.02) = +0.02
全局距离 = -1 × (+0.02) = -0.02 < 0

策略违反！原因：降落伞在飞行器爬升时被释放（违反条件 3）
```

## 3.5 Bug 后处理（原文 Section V-C）

### 3.5.1 目的

> "PGFUZZ conducts Bug Post-Processing to find the minimized sequence of inputs that causes a policy violation. The minimized sequence can be later used to analyze the violation's root cause."

### 3.5.2 输入最小化算法

```
输入最小化步骤：

1. 创建新进程执行单独的模拟器

2. 从原始输入序列 input(1,...,n) 中排除一个输入 input_i
   创建新序列 input(1,...,i-1,i+1,...,n)

3. 在模拟器上执行新序列

4. 如果新序列不导致相同的策略违反：
   → input_i 是必须的，保留
   如果新序列仍然导致策略违反：
   → input_i 不是必须的，可删除

5. 重复步骤 2-4 直到找到最小输入序列
```

### 3.5.3 示例

原文中的降落伞策略示例：

> "For example, the input sequence {(mode=ACRO),(wind=5),(parachute=on)} violates A.CHUTE1 policy. However, (wind=5) does not contribute to the policy violation."

```
原始序列：{(mode=ACRO), (wind=5), (parachute=on)} → 违反策略

测试去掉 (wind=5)：
{(mode=ACRO), (parachute=on)} → 仍然违反策略

最小化结果：{(mode=ACRO), (parachute=on)}
```

---

# 第四部分：实验评估

## 4.1 实验设置（原文 Section VII 和 Table V）

### 4.1.1 测试目标

| RV 系统 | 版本 | 测试对象 | 模拟器 |
|---------|------|----------|--------|
| ArduPilot | 4.0.3 | Quadrotor（四旋翼） | APM SITL, Gazebo |
| PX4 | 1.9 | Quadrotor | JSBSim, Gazebo |
| Paparazzi | 5.16 | Quadrotor | NPS, Gazebo |

### 4.1.2 测试配置

- 测试时长：48 小时
- 硬件：Intel Core i7-7700 CPU @ 3.6 GHz，32 GB RAM
- 系统：Ubuntu 18.04 64-bit
- 策略数量：56 条（ArduPilot 30 条，PX4 21 条，Paparazzi 5 条）

### 4.1.3 测试策略示例（原文 Table VI）

| ID | 描述 |
|----|------|
| A.ALT_HOLD2 / PP.HOVERZ | 如果油门杆在中间位置（1,500），飞行器必须保持当前高度 |
| A.FLIP1 | 只有当横滚角 < 45°、油门 ≥ 1,500、高度 > 10米、且当前飞行模式是 ACRO 或 ALT_HOLD 时，才能切换到 FLIP 模式 |
| A.GPS.FS1 | 当检测到的 GPS 卫星数少于四颗时，飞行器必须触发 GPS fail-safe 模式 |
| A.LOITER1 / PX.HOLD1 / PP.HOVERC | 飞行器必须保持恒定的位置、航向和高度 |
| A.CHUTE1 | 只有满足以下条件才能释放降落伞：(1) 电机已启动，(2) 不在 FLIP 或 ACRO 模式，(3) 气压计显示飞行器没有在爬升，(4) 当前高度高于 CHUTE_ALT_MIN 参数值 |
| A.RC.FS1 | 只有当飞行器在 ACRO 模式下已启动且油门输入低于最小值（FS_THR_VALUE 参数）时，飞行器必须立即解除启动 |
| A.RC.FS2 | 如果油门输入低于 FS_THR_VALUE 参数，必须将当前模式更改为 RC fail-safe 模式 |
| ALIVE | 飞行器必须每 k 秒向地面控制系统发送心跳消息（适用于 A/PX/PP） |
| PX.GPS.FS1 | 如果检测到 GPS 丢失后超过 COM_POS_FS_DELAY 秒，必须触发 GPS fail-safe |
| PX.GPS.FS2 | 如果触发了 GPS fail-safe 且遥控器可用，飞行模式必须更改为 ALTITUDE 模式 |
| PX.TAKEOFF1 | 当飞行器执行起飞命令时，目标高度必须是 MIS_TAKEOFF_ALT 参数值 |

## 4.2 主要结果（原文 Table VII）

### 4.2.1 Bug 发现统计

| 策略 | Bug 数 | 根本原因分类 | 物理影响分类 |
|------|--------|-------------|-------------|
| | | 范围过宽 / 实现错误 / 未实现 / 无检查 | 坠机 / 软件崩溃 / 姿态不稳 / 意外行为 |
| **ArduPilot** |
| A.ALT_HOLD2 | 7 | 2 / 5 / 0 / 0 | 4 / 0 / 0 / 3 |
| A.FLIP1 | 1 | 0 / 1 / 0 / 0 | 1 / 0 / 0 / 0 |
| A.FLIP1 & A.ALT_HOLD2 | 1 | 0 / 1 / 0 / 0 | 1 / 0 / 0 / 0 |
| A.GPS.FS2 | 1 | 0 / 1 / 0 / 0 | 0 / 0 / 0 / 1 |
| A.LOITER1 | 8 | 2 / 1 / 5 / 0 | 1 / 0 / 4 / 3 |
| A.CHUTE1 | 1 | 0 / 1 / 0 / 0 | 1 / 0 / 0 / 0 |
| A.RC.FS1 | 1 | 0 / 1 / 0 / 0 | 0 / 0 / 0 / 1 |
| A.RC.FS2 | 1 | 0 / 1 / 0 / 0 | 0 / 0 / 0 / 1 |
| A.ALIVE | 82 | 5 / 0 / 0 / 77 | 0 / 82 / 0 / 0 |
| **ArduPilot 小计** | **103** | 4 / 17 / 0 / 82 | 6 / 82 / 4 / 11 |
| **PX4** |
| PX.ALIVE | 8 | 0 / 0 / 8 / 0 | 0 / 8 / 0 / 0 |
| PX.GPS.FS1 | 2 | 0 / 2 / 0 / 0 | 0 / 0 / 0 / 2 |
| PX.GPS.FS2 | 2 | 0 / 2 / 0 / 0 | 0 / 0 / 0 / 2 |
| PX.HOLD1 | 23 | 20 / 1 / 2 / 0 | 9 / 0 / 13 / 1 |
| PX.TAKEOFF1 | 1 | 0 / 1 / 0 / 0 | 0 / 0 / 0 / 1 |
| **PX4 小计** | **36** | 20 / 6 / 8 / 2 | 9 / 8 / 13 / 6 |
| **Paparazzi** |
| PP.HOVERC | 10 | 10 / 0 / 0 / 0 | 4 / 0 / 0 / 6 |
| PP.HOVERZ | 7 | 7 / 0 / 0 / 0 | 1 / 0 / 2 / 4 |
| **Paparazzi 小计** | **17** | 17 / 0 / 0 / 0 | 5 / 0 / 8 / 4 |
| **总计** | **156** | **41 / 23 / 8 / 84** | **20 / 90 / 25 / 21** |

### 4.2.2 Bug 根本原因分类（原文 Section VII-B）

| 类别 | 数量 | 说明 |
|------|------|------|
| **Broad valid range（范围过宽）** | 41 | 配置参数的有效范围设置不正确。例：ATC_RATE_R_MAX 有效范围是 0-1080，但当用户设置小于 100 时，会导致姿态控制不稳定并坠毁 |
| **Misimplementation（实现错误）** | 23 | 功能在正常或特定情况下未正确工作。例：PX4 在特定飞行模式下无法触发 GPS fail-safe |
| **Unimplemented（未实现）** | 8 | 文档中提到的传感器故障处理条件未实现（主要在 PX4） |
| **No checking valid range（无范围检查）** | 84 | 完全没有检查配置参数的有效范围。例：当 ATC_RATE_R_MAX 参数被赋予超出预定义范围的值时，产生浮点异常 |

### 4.2.3 物理影响分类

| 影响类型 | 数量 | 说明 |
|----------|------|------|
| **Crash on the ground（坠机）** | 20 | 飞行器失去姿态控制并向 GCS 发送自由落体警告消息 |
| **Software crash（软件崩溃）** | 90 | 飞行控制软件因浮点异常而崩溃 |
| **Unstable attitude（姿态不稳）** | 25 | 飞行器姿态波动 |
| **Unexpected behavior（意外行为）** | 21 | 其他问题，包括未检查前置条件改变飞行器状态、未能保持相同高度、特技飞行后高度计算错误、未能触发 fail-safe 模式 |

### 4.2.4 Bug 分析（原文 Section VII-B - Analysis of Bugs）

| Bug 类型 | 数量 | 百分比 |
|----------|------|--------|
| 逻辑 Bug（misimplemented + unimplemented） | 31 | 19.9% |
| 内存安全 Bug（导致内存损坏的输入验证 bug） | 90 | 57.7% |
| 输入验证 Bug（不导致内存损坏） | 35 | 22.4% |
| 无害 Bug（不导致崩溃、不稳定姿态或错误高度） | 11 | 7.05% |

### 4.2.5 责任披露结果（原文 Table IX）

| 系统 | Bug 数 | 已确认 | 将修复 | 已修复 |
|------|--------|--------|--------|--------|
| ArduPilot | 103 | 79 | 5 | 3 |
| PX4 | 36 | 27 | 21 | 6 |
| Paparazzi | 17 | 0 | 0 | 0 |
| **总计** | **156** | **106** | **26** | **9** |

## 4.3 组件评估

### 4.3.1 Profiling Engine 效果（原文 Table III 和 Table IV）

**配置参数（InputP）空间缩减**：

| RV 系统 | 原始 InputP 数 | 静态分析后 | 动态分析后 | 缩减比例 |
|---------|---------------|------------|------------|----------|
| ArduPilot | 1,140 | 268 | 209 | 81.7% |
| PX4 | 579 | 333 | 176 | 69.6% |
| Paparazzi | 82 | 57 | 51 | 37.8% |

**用户命令和环境因素空间缩减**：

| RV 系统 | InputC 数 | InputE 数 | 动态分析后 | 缩减比例 |
|---------|-----------|-----------|------------|----------|
| ArduPilot | 58 | 168 | 150 | 33.6% |
| PX4 | 66 | 30 | 43 | 55.2% |
| Paparazzi | 116 | 8 | 46 | 62.9% |

### 4.3.2 噪声消除效果（原文 Figure 7）

论文展示了在 LOITER 飞行模式下，使用 10 m/s 风速、60°风向、3 m/s² 加速度噪声的噪声消除效果。移动平均窗口宽度设为 4。

### 4.3.3 触发 Bug 所需的输入类型（原文 Table VIII）

| 输入类型组合 | Bug 数量 | 百分比 |
|--------------|----------|--------|
| 只需要 InputP | 120 | 76.9% |
| 只需要 InputC | 10 | 6.4% |
| 只需要 InputE | 2 | 1.3% |
| InputP + InputC | 20 | 12.8% |
| InputP + InputE | 3 | 1.9% |
| InputP + InputC + InputE | 1 | 0.6% |
| **总计** | **156** | **100%** |

**关键发现**：77%（120/156）的 bug 只需要改变配置参数即可触发。

### 4.3.4 与无策略引导 Fuzzing 的对比（原文 Figure 8）

48 小时内发现的 bug 数量对比：

| Fuzzing 方法 | 发现 Bug 数 |
|--------------|-------------|
| 完整 PGFUZZ 方法 | **156** |
| 随机 fuzzing + 输入空间最小化 | 63 |
| 随机 fuzzing + 完整输入空间 | 36 |
| 随机 fuzzing + 排除的输入空间 | 21 |

**结论**：
- 策略引导非常重要：156 vs 63
- 输入空间缩减也重要：63 vs 36

### 4.3.5 与 RVFuzzer 的对比（原文 Section VII-B）

> "We contacted RVFuzzer's authors to determine how many of the bugs reported by PGFUZZ can be discovered by RVFuzzer. RVFuzzer could find 28 out of 156 bugs."

| 对比项 | 数量 |
|--------|------|
| PGFUZZ 发现的 bug 总数 | 156 |
| RVFuzzer 也能发现的 | 28 |
| **只有 PGFUZZ 能发现的** | **128 (82%)** |

**RVFuzzer 无法发现 128 个 bug 的三个原因**：

> "First, if a policy violation does not affect the vehicle's attitude and flight path, RVFuzzer cannot detect the violation because RVFuzzer only uses one policy that defines the stable attitude and a correct flight path."

> "Second, some bugs are only disclosed with user commands, environmental factors, and configuration parameters. However, RVFuzzer only mutates inputs for the configuration parameters."

> "Lastly, RVFuzzer cannot discover a set of bugs due to its limited binary search-based algorithm."

| 原因 | 说明 |
|------|------|
| **原因 1** | 如果策略违反不影响飞行器姿态和飞行路径，RVFuzzer 无法检测（它只使用定义稳定姿态和正确飞行路径的策略） |
| **原因 2** | 有些 bug 只能通过用户命令、环境因素和配置参数的组合触发，但 RVFuzzer 只变异配置参数 |
| **原因 3** | RVFuzzer 的二分搜索算法有局限性（例如 PSC_ACC_XY_FILT 参数在端点值 0 和 2.0 时正常，但在 0.0001 时导致崩溃） |

## 4.4 Case Study（原文 Section VII-C）

### 4.4.1 Case Study 1：未检查前置条件导致的意外行为

**策略**：A.CHUTE1（降落伞释放条件）

**发现的 bug**：

> "PGFUZZ discovered that ArduPilot only checks the last condition among the four preconditions when the parachute is manually released."

ArduPilot 只检查第 4 个条件（高度），而忽略了前 3 个条件。

**攻击场景**（原文 Figure 9）：

```
攻击步骤：
1. 触发 FLIP 模式（翻滚特技）
2. 同时发送释放降落伞命令

结果：
- 正常情况（ALT_HOLD 模式）：Roll 和 Pitch 平稳，安全降落
- 攻击情况（FLIP 模式）：失去俯仰控制，28秒时坠毁

检测方式：飞行控制软件在检测到不稳定姿态着陆时向 GCS 发送坠毁警告消息
```

### 4.4.2 Case Study 2：翻滚后无法保持高度

**策略**：A.ALT_HOLD2（在 ALT_HOLD 模式下保持高度）

**发现的 bug**（原文 Figure 10）：

> "PGFUZZ discovered that this requirement is not correctly implemented if roll axis rate controller parameters are changed."

当两个横滚轴速率控制器参数（ATC_RATE_RLL_FF 和 ATC_RATE_R_MAX）被修改时，ArduPilot 在翻滚机动后无法保持高度。

**根本原因**：

> "The root cause is the broad range of accepted parameter values."

参数有效范围设置过宽。

**攻击场景**：

```
攻击步骤：
1. 将两个横滚轴速率控制器参数设为较小值
2. 等待用户触发 FLIP 模式

结果：
- 正常情况：FLIP 模式后能恢复到 ALT_HOLD 并保持高度
- 攻击情况：由于横滚角速度受限，无人机无法恢复稳定横滚角，
           导致无法保持高度，最终坠毁

隐蔽性：修改的横滚参数在正常操作中不影响姿态控制（不需要大横滚角速度），
        用户不会注意到受限的横滚角速度
```

### 4.4.3 Case Study 3：特技飞行后高度计算错误

**策略**：A.ALT_HOLD2

**发现的 bug**（原文 Figure 10d 和 Figure 11）：

> "PGFUZZ discovered that ArduPilot incorrectly computes the altitude when high deviations in GPS sensor occur."

当 GPS 传感器出现高偏差时，ArduPilot 错误计算高度。

**根本原因**：

1. 高 GPS 偏差导致 ArduPilot 将高度测量源从 GPS 切换到气压计
2. ArduPilot 错误地应用 GND_ALT_OFFSET 参数计算气压高度

**攻击场景**：

```
攻击步骤：
1. 配置 FS_EKF_ACTION 参数：GPS 偏差时着陆
2. 设置较大的 GND_ALT_OFFSET 和 LAND_SPEED_HIGH 参数
3. 等待用户执行特技飞行（如 FLIP 模式）

结果：
- 特技飞行导致 GPS 偏差
- 触发着陆程序
- 由于高度计算错误，无人机持续使用 LAND_SPEED_HIGH（高速下降）
  而非 LAND_SPEED（低速下降）
- 以 12.86 m/s 的速度撞地

隐蔽性：
- 参数变更不影响正常操作
- bug 在用户执行特技飞行时才触发
```

### 4.4.4 Case Study 4：GPS Fail-safe 未触发

**策略**：PX.GPS.FS1

**发现的 bug**（原文 Figure 12）：

> "PGFUZZ discovered that assigning a negative value to the COM_POS_FS_DELAY parameter, which represents the time delay in turning on a GPS fail-safe and setting to specific flying modes cause PX4 to fail to trigger the GPS fail-safe."

将 COM_POS_FS_DELAY 参数设为负值会导致 PX4 在 ORBIT 模式或飞向某位置时无法触发 GPS fail-safe。

**根本原因**：

> "The violation happens because PX4 developers do not implement a parameter range check. PX4 v1.7.4 forces COM_POS_FS_DELAY parameter to have a value in the valid range. Thereafter, it checks whether the GPS fail-safe needs to be triggered. However, we found that the code lines to check the COM_POS_FS_DELAY parameter are removed by developers in PX4 v1.9 while updating the fail-safe code snippets."

PX4 v1.9 更新 fail-safe 代码时删除了参数范围检查代码。

**攻击场景**：

```
攻击步骤：
1. 将 COM_POS_FS_DELAY 设为负值
2. 等待无人机进入 GPS 信号弱的区域

结果：
- GPS 信号丢失
- Fail-safe 未触发
- 无人机继续使用惯性导航（IMU）
- 由于 IMU 累积误差，无人机随风向漂移
- 偏离预定路线最多 70.5 米（经度）和 20.7 米（纬度）

隐蔽性：GPS 信号丢失是正常现象（尤其在高度城市化区域）
```

---

# 第五部分：PGFUZZ vs RVFuzzer 对比总结

## 5.1 核心差异对比表

| 对比维度 | RVFuzzer | PGFUZZ |
|----------|----------|--------|
| **发表时间** | USENIX Security 2019 | NDSS 2021 |
| **Bug Oracle** | 控制不稳定（IAE 公式） | 策略违反（距离度量） |
| **策略表达** | 无（只检测不稳定） | MTL 公式（可表达复杂安全规则） |
| **输入类型** | 主要是配置参数 (InputP)，有限的环境因素 (InputE) | 配置参数 + 用户命令 + 环境因素 |
| **变异策略** | 二分搜索 | 基于命题距离的引导 |
| **输入空间缩减** | 无自动缩减 | 静态分析 + 动态分析自动缩减 |
| **适用场景** | 参数范围验证 bug | 策略违反 bug（包括但不限于不稳定） |
| **发现能力** | 28/156 (18%) | 156/156 (100%) |

## 5.2 Bug 检测能力对比

### RVFuzzer 能检测的 Bug 类型

- 参数值导致的控制不稳定
- 飞行路径偏离
- 物理崩溃（伴随明显不稳定）

### PGFUZZ 额外能检测的 Bug 类型

1. **不导致不稳定的策略违反**
   - GPS fail-safe 未触发（无人机"稳定"地飞向错误方向）
   - 降落伞条件检查缺失

2. **需要用户命令触发的 Bug**
   - FLIP 模式下释放降落伞

3. **二分搜索无法发现的 Bug**
   - 参数在端点正常但中间值有问题（如 PSC_ACC_XY_FILT）

## 5.3 方法论对比

### 5.3.1 Bug Oracle 对比

```
RVFuzzer Bug Oracle：
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│   IAE 公式：deviation(t) = ∫[t→t+w] |r(s)-x(s)|/w ds       │
│                                                             │
│   如果 deviation(t) > τ → 检测到不稳定                      │
│                                                             │
│   局限：只能检测"控制不稳定"                                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘

PGFUZZ Bug Oracle：
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│   全局距离 = 基于 MTL 公式计算的距离度量                     │
│                                                             │
│   如果 全局距离 < 0 → 检测到策略违反                        │
│                                                             │
│   优势：可检测任何可用 MTL 表达的策略违反                    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 5.3.2 变异策略对比

```
RVFuzzer 变异策略：
┌─────────────────────────────────────────────────────────────┐
│   二分搜索：                                                │
│   - 单调性假设：不稳定随参数值单调变化                       │
│   - 高效找到有效范围边界                                    │
│   - 局限：无法发现"中间值"问题                              │
└─────────────────────────────────────────────────────────────┘

PGFUZZ 变异策略：
┌─────────────────────────────────────────────────────────────┐
│   命题距离引导：                                             │
│   - 追踪每个命题距离的变化                                  │
│   - 记住有效的 (input, value) 配对                          │
│   - 优势：可发现复杂的参数组合问题                          │
└─────────────────────────────────────────────────────────────┘
```

---

# 第六部分：PGFUZZ 的局限性

## 6.1 作者承认的局限性（原文 Section IX）

### 6.1.1 RV 模拟器的不完美

> "We use Software-in-the-Loop (SITL) as our testing environment. Imperfect simulations could cause two issues."

**问题 1：假阳性**
> "First, if simulators incorrectly simulate the vehicle's states and/or hardware, PGFUZZ will identify false-positive policy violations."

如果模拟器错误模拟飞行器状态或硬件，PGFUZZ 会报告假阳性。

**作者的回应**：
> "However, we confirmed that all policy violations found by PGFUZZ could be reproduced on a real vehicle. We used a 3DR IRIS+ UAV platform equipped with the Pixhawk 1 flight management unit board in our experiments."

所有发现的策略违反都在真实飞行器上得到验证（3DR IRIS+ UAV，Pixhawk 1）。

**问题 2：覆盖范围限制**
> "Second, if the simulators do not support specific hardware (e.g., RFD 900 radio modem), PGFUZZ cannot find bugs in those hardware modules."

模拟器不支持特定硬件时，PGFUZZ 无法发现这些硬件模块中的 bug。

**可能的解决方案**：
> "To address this, PGFUZZ can be integrated into Hardware-in-the-loop (HIL) simulation or Simulation-In-Hardware (SIH) where firmware is run on real flight controller hardware."

可以集成到 HIL 或 SIH 仿真中。

### 6.1.2 时序逻辑的实时监控限制

> "Since PGFUZZ checks policies at run-time during a simulation, at time point t, only the data traces for 1,...,t are available to check the policies. Therefore, MTL policies with unbounded future operators cannot be checked at run-time."

由于运行时检查策略，在时间点 t 只有 1...t 的数据可用，因此无法检查带有无界未来算子的 MTL 策略。

**解决方案**：
> "Following the online monitoring systems, we define the policies with a subclass of MTL that considers unbounded past and bounded future."

使用考虑无界过去和有界未来的 MTL 子类。

**示例**：
```
不可检查：◇(ALT > 10)  （高度最终会超过 10 米 - 无界未来）

可检查：◇_{[0,5]}(ALT > 10)  （高度在 5 秒内会超过 10 米 - 有界未来）
```

### 6.1.3 移植到其他 RV 的工作量

> "Users can port PGFUZZ to other types of RV software by following six steps."

移植步骤：
1. 为 RV 创建 MTL（或 LTL）策略
2. 识别 PGFUZZ 状态列表中未包含的新状态
3. 更新同义词表
4. 将 MTL 公式术语映射到源代码中的变量
5. 根据新 MTL 公式验证和更新策略违反谓词
6. 排除导致假阳性的自毁性输入

**移植时间统计**：
> "Specifically, we deployed PGFUZZ in the order of ArduPilot, PX4, and Paparazzi, and the manual effort took 13.5, 6.3, and 3.6 hours, respectively."

| 系统 | 移植时间 |
|------|----------|
| ArduPilot | 13.5 小时 |
| PX4 | 6.3 小时 |
| Paparazzi | 3.6 小时 |
| **总计** | **23.4 小时** |

从 ArduPilot 移植到 PX4 的具体工作：
- 修改 54 LoC（预处理阶段）
- 修改 94 LoC（变异引擎，以适应 MAVLink 协议差异）

## 6.2 其他可观察到的局限性

### 6.2.1 策略提取的人工成本

论文中策略提取是手动进行的：
- ArduPilot：7.5 小时（两位作者），30 条策略
- PX4：3.5 小时，21 条策略
- Paparazzi：2.4 小时，5 条策略

这需要领域专业知识和对文档/代码的深入理解。

### 6.2.2 策略完整性问题

论文测试的 56 条策略可能不完整。文档中未明确记录的安全要求可能被遗漏。

### 6.2.3 单变量变异策略

根据 Algorithm 1，PGFUZZ 每次只变异一个输入：
```
15: input ← RANDOM(Input_min, MAP)  // 随机选一个输入
```

这可能无法有效发现需要同时变异多个参数才能触发的 bug。

---

# 第七部分：学习总结与思考

## 7.1 论文的核心贡献回顾

1. **MTL 策略表达**：将自然语言的安全需求形式化为可计算机检查的逻辑公式

2. **距离度量引导**：通过命题距离和全局距离量化"离策略违反有多远"，指导 fuzzing 方向

3. **三阶段架构**：预处理（缩减输入空间）→ 策略引导 Fuzzing → Bug 后处理（输入最小化）

4. **大规模实证验证**：在三个主流飞控软件上发现 156 个真实 bug

## 7.2 从 RVFuzzer 到 PGFUZZ 的演进

```
RVFuzzer (2019)                    PGFUZZ (2021)
─────────────                      ────────────
检测"不稳定"        ─────────▶     检测"策略违反"
单一 Bug Oracle     ─────────▶     可定制的 MTL 策略
主要测参数          ─────────▶     参数 + 命令 + 环境
二分搜索            ─────────▶     距离引导变异
发现 89 个 bug      ─────────▶     发现 156 个 bug
```

## 7.3 关键技术点总结

### 7.3.1 MTL 公式的构造

```
模板选择 → 术语识别 → 公式构造 → 时间约束确定

示例：降落伞策略
1. 选择模板 T2（条件蕴含）
2. 识别术语：Parachute, Armed, Mode, ALT, CHUTE_ALT_MIN
3. 构造公式：□{(Parachute=on) → (Armed=true) ∧ ...}
4. 时间约束：无（使用 □ 表示任意时刻）
```

### 7.3.2 距离度量的设计

```
命题距离：
- 布尔值：+1（真）/ -1（假）
- 数值：归一化差值

全局距离：
- AND → min()
- OR → max()
- NOT → × (-1)

判定规则：
- 全局距离 > 0：策略满足
- 全局距离 < 0：策略违反
```

### 7.3.3 变异引擎的反馈机制

```
关键点：追踪每个命题距离，而非只看全局距离

反馈逻辑：
1. 如果某个输入导致任意 Pi 增加 → 记住 (input, value)
2. 下次选到同一输入时使用记住的值
3. 最终当足够多的 Pi > 0 时，全局距离变成负数
```

## 7.4 与博后合作的准备要点

基于对 RVFuzzer 和 PGFUZZ 两篇论文的学习，以下是可能的研究方向：

1. **策略自动提取**：减少人工构造 MTL 公式的成本

2. **多变量协同变异**：探索同时变异多个参数的策略

3. **跨平台通用性**：减少移植到新 RV 系统的工作量

4. **实时 Fuzzing**：在真实飞行中进行策略监控

5. **与机器学习结合**：使用 ML 预测有效的变异方向

---

# 附录

## 附录 A：论文中的完整策略列表（原文 Table XII 和 Appendix E）

详见论文 Table XII，包含 56 条策略的完整 MTL 表示。

## 附录 B：实现细节（原文 Section VI）

| 组件 | 实现细节 |
|------|----------|
| 通信协议 | MAVLink v2.0（通过 Pymavlink v2.4.9 和 PPRZLINK v2.0） |
| 静态分析 | LLVM 9.0.0 + Static Value-Flow Analysis tool |
| 动态分析 | 586 LoC Python（ArduPilot）+ 741 LoC Python（Paparazzi） |
| 变异引擎 | 1,379 LoC Python |
| Bug 后处理 | 626 LoC Python（ArduPilot/PX4）+ 794 LoC（Paparazzi） |

## 附录 C：物理状态列表（原文 Table XI）

| ID | 类型 | 状态 | 描述 |
|----|------|------|------|
| S1 | Position | latitude, longitude, altitude | 飞行器的 (x,y,z) 位置 |
| S2 | Attitude | roll, pitch, yaw, roll speed, pitch speed, yaw speed, reference roll/pitch/yaw | 测量和期望的姿态 |
| S3 | Operation | air speed, ground speed, throttle, climb rate, reference air speed, flight mode, parachute | 物理运动和操作模式 |
| S4 | RC inputs | RC 1-4 | 用户的无线电通道输入 |
| S5 | System | system clock, flight status, mission, pre-arm checking | 系统通用信息 |
| S6 | Sensor | gyroscope, accelerometer, magnetometer, barometer, GPS | 传感器状况 |

---

# 参考文献

本文档基于以下论文：

1. Kim, H., Ozmen, M. O., Bianchi, A., Celik, Z. B., & Xu, D. (2021). **PGFUZZ: Policy-Guided Fuzzing for Robotic Vehicles**. In Network and Distributed Systems Security (NDSS) Symposium 2021.

2. Kim, T., Kim, C. H., Rhee, J., Fei, F., Tu, Z., Walkup, G., ... & Xu, D. (2019). **RVFuzzer: Finding Input Validation Bugs in Robotic Vehicles through Control-Guided Testing**. In 28th USENIX Security Symposium.
